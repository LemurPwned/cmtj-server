#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <chrono>

#include <future>
#include <map>
#include <thread>

#include "./third/drivers.hpp"
#include "./third/httplib.h"
#include "./third/junction.hpp"

#include "DatabaseHandler.hpp"
#include "QueueHandler.hpp"
#include "TaskParser.hpp"

#include <cmath>
#include <exception>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <tuple>

using json = nlohmann::json;

void setupLogger()
{

    spdlog::info("Setting up the logger");
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug

    // change log pattern
    spdlog::set_pattern("[%H:%M:%S %z] [%^--%L--%$] [Th.%t] %v");
}

class Engine
{
private:
    SafeQueue<json> queue;
    TaskParser parser;
    TaskDB saver;

    json parseRunPIM(Junction &j, json pimTask)
    {
        spdlog::info("Parsing the PIM task");
        json subTaskDef = pimTask.at("parameters");
        const auto hMin = subTaskDef.at("Hmin").get<double>();
        const auto hMax = subTaskDef.at("Hmax").get<double>();
        const int hsteps = subTaskDef.at("Hsteps").get<int>();

        const auto HoePulseAmplitude = subTaskDef.at("HOePulseAmplitude").get<double>();
        const auto HoeDir = subTaskDef.at("HOedir").get<std::vector<double>>();
        const auto pulseStart = subTaskDef.at("pulseStart").get<double>();
        const auto pulseStop = subTaskDef.at("pulseStop").get<double>();

        const auto theta = subTaskDef.at("theta").get<double>() * M_PI / 180;
        const auto phi = subTaskDef.at("phi").get<double>() * M_PI / 180;

        const double time = subTaskDef.at("time").get<double>();
        const double tStep = subTaskDef.at("tStep").get<double>();
        const double tWrite = subTaskDef.at("tWrite").get<double>();
        const double tStart = subTaskDef.at("tStart").get<double>();

        return runPIM(pimTask.at("uuid").get<std::string>(), j,
                      HoePulseAmplitude, pulseStart, pulseStop, CVector(HoeDir),
                      hMin, hMax, theta, phi, hsteps,
                      time, tStep, tWrite, tStart);
    }

    json runPIM(std::string uuid, Junction &mtj,
                double HoePulseAmplitude, double pulseStart, double pulseStop, CVector HoeDir,
                double hMin, double hMax, double theta, double phi, int hsteps,
                double time, double tStep, double tWrite, double tStart)
    {

        typedef std::tuple<double, std::map<std::string, std::vector<double>>> intermediateRes;
        const double hstep = (hMax - hMin) / hsteps;
        spdlog::info("Calculating the PIM");
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // distribute the frequencies
        const int threadNum = std::thread::hardware_concurrency() - 1;
        spdlog::debug("Using {} threads to distribute the task workload!", threadNum);

        std::vector<std::future<std::vector<intermediateRes>>> threadResults;
        threadResults.reserve(threadNum);

        const int minThreadLoad = (int)hsteps / threadNum;
        int extraThreadWorkload = hsteps % threadNum;
        spdlog::debug("Min subtask load {} per thread", minThreadLoad);
        int lastThreadLoadIndx = 0;

        for (int t = 0; t < threadNum; t++)
        {
            int threadLoad = minThreadLoad;
            if (extraThreadWorkload)
            {
                threadLoad += 1;
                // TODO: think about this -- whether we actually want an even workload
                extraThreadWorkload--;
            }
            const double threadMinH = hMin + lastThreadLoadIndx * hstep;
            lastThreadLoadIndx += threadLoad;
            const double threadMaxH = hMin + lastThreadLoadIndx * hstep;

            spdlog::debug("Launching thread {} with Hs => {} to {}, load {}", t, threadMinH, threadMaxH, threadLoad);
            threadResults.emplace_back(std::async([=]() mutable {
                std::vector<intermediateRes> resAcc;
                int freqIndx = 0;
                for (double hAmplitude = threadMinH; hAmplitude < threadLoad; hAmplitude += hstep)
                {
                    const AxialDriver HDriver(
                        ScalarDriver::getConstantDriver(hAmplitude * sin(theta) * cos(phi)),
                        ScalarDriver::getConstantDriver(hAmplitude * sin(theta) * sin(phi)),
                        ScalarDriver::getConstantDriver(hAmplitude * cos(theta)));

                    AxialDriver HoeDriver(
                        ScalarDriver::getStepDriver(0, HoePulseAmplitude, pulseStart, pulseStop),
                        ScalarDriver::getStepDriver(0, HoePulseAmplitude, pulseStart, pulseStop),
                        ScalarDriver::getStepDriver(0, HoePulseAmplitude, pulseStart, pulseStop));

                    HoeDriver.applyMask(HoeDir);

                    mtj.clearLog();
                    mtj.setLayerExternalFieldDriver(
                        "all",
                        HDriver);
                    mtj.setLayerOerstedFieldDriver(
                        "all",
                        HoeDriver);
                    mtj.runSimulation(
                        time,
                        tStep, tWrite, false, false, false);

                    auto resTuple = std::make_tuple(hAmplitude, mtj.spectralFFT(tStart, tStep));
                    resAcc.push_back(std::move(resTuple));
                }
                return resAcc;
            }));
        }
        json finalRes;
        const std::vector<std::string> mags = {"x", "y", "z"};
        for (auto &result : threadResults)
        {
            for (auto [hAmpl, resMap] : result.get())
            {
                json subresult;
                subresult["hAmpl"] = std::move(hAmpl);
                for (const auto &mag : mags)
                {
                    subresult[mag + "_amplitude"] = std::move(resMap[mag + "_amplitude"]);
                    subresult[mag + "_phase"] = std::move(resMap[mag + "_phase"]);
                }
                finalRes["results"].push_back(std::move(subresult));
            }
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        spdlog::info("Total result retrieval time = {} [s]", std::chrono::duration_cast<std::chrono::seconds>(end - begin).count());

        return finalRes;
    }

    json parseRunVSD(Junction &j, json vsdTask)
    {
        spdlog::info("Parsing the VSD task");
        json subTaskDef = vsdTask.at("parameters");
        const double phase = subTaskDef.at("phase").get<double>();

        const auto hMin = subTaskDef.at("Hmin").get<double>();
        const auto hMax = subTaskDef.at("Hmax").get<double>();
        const int hsteps = subTaskDef.at("Hsteps").get<int>();

        const auto Hoe = subTaskDef.at("HOe").get<double>();
        const auto HoeDir = subTaskDef.at("HOedir").get<std::vector<double>>();

        // convert the angle to radians
        const auto theta = subTaskDef.at("theta").get<double>() * M_PI / 180;
        const auto phi = subTaskDef.at("phi").get<double>() * M_PI / 180;

        const int fsteps = subTaskDef.at("fsteps").get<int>();
        double fstart = subTaskDef.at("fmin").get<double>();
        double fstop = subTaskDef.at("fmax").get<double>();

        const double time = subTaskDef.at("time").get<double>();
        const double tStep = subTaskDef.at("tStep").get<double>();
        const double tWrite = subTaskDef.at("tWrite").get<double>();
        const double tStart = subTaskDef.at("tStart").get<double>();
        const double power = subTaskDef.at("power").get<double>();

        return runVSD(vsdTask.at("uuid").get<std::string>(), // task UUID
                      j,                                     // junction
                      Hoe,
                      CVector(HoeDir), // start ext field vector
                      hMin,
                      hMax,
                      theta,  // out of plane angle
                      phi,    // in plane angle
                      hsteps, // step ext field vector
                      phase,  // phase of the excitation
                      fstart, // start frequency
                      fstop,  // stop frequency
                      fsteps, // number of freq steps
                      time,
                      tStep,
                      tWrite,
                      tStart,
                      power);
    }

    json runVSD(std::string uuid, Junction &mtj, double Hoe, CVector HoeDir, double hMin,
                double hMax, double theta, double phi, int hsteps,
                double phase,
                double fstart, double fstop, int fsteps, double time,
                double tStep, double tWrite, double tStart, double power)
    {

        typedef std::tuple<double, std::map<std::string, double>> fnRes;
        typedef std::tuple<double, double, double, CVector, CVector> multituple;

        double fstep = ((fstop - fstart) / fsteps);
        double hstep = ((hMax - hMin) / hsteps);
        spdlog::info("Calculating the VSD");
        spdlog::debug("Fmin {}, Fmax {}, #steps {}, step {} MHz", fstart, fstop, fsteps, (fstep / 1e6));
        spdlog::debug("Hmin {}, Hmax {}, #steps {}, step {}", hMin, hMax, hsteps, hstep);
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // distribute the frequencies
        const int threadNum = std::thread::hardware_concurrency() - 1;
        spdlog::debug("Using {} threads to distribute the task workload!", threadNum);

        std::vector<std::future<std::vector<multituple>>> threadResults;
        threadResults.reserve(threadNum);

        const int minThreadLoad = (int)fsteps / threadNum;
        int extraThreadWorkload = fsteps % threadNum;
        int lastThreadLoadIndx = 0;
        spdlog::debug("Min subtask load {} per thread", minThreadLoad);
        for (int t = 0; t < threadNum; t++)
        {
            int threadLoad = minThreadLoad;
            if (extraThreadWorkload)
            {
                threadLoad += 1;
                // TODO: think about this -- whether we actually want an even workload
                // extraThreadWorkload--;
            }
            const double threadMinFreq = fstart + lastThreadLoadIndx * fstep;
            lastThreadLoadIndx += threadLoad;
            const double threadMaxFreq = fstart + lastThreadLoadIndx * fstep;

            spdlog::info("Launching thread {} with freq => {} MHz to {} MHz, load {}", t,
                         (threadMinFreq / 1e6),
                         (threadMaxFreq / 1e6),
                         threadLoad);
            threadResults.emplace_back(std::async([=]() mutable {
                std::vector<multituple> resAcc;

                int freqIndx = 0;
                for (double freq = threadMinFreq; freqIndx < threadLoad; freq += fstep)
                {
                    const AxialDriver HoeDriver(
                        ScalarDriver::getSineDriver(0, HoeDir.x * Hoe, freq, phase),
                        ScalarDriver::getSineDriver(0, HoeDir.y * Hoe, freq, phase),
                        ScalarDriver::getSineDriver(0, HoeDir.z * Hoe, freq, phase));

                    for (double hAmplitude = hMin; hAmplitude < hMax; hAmplitude += hstep)
                    {
                        const AxialDriver HDriver(
                            ScalarDriver::getConstantDriver(hAmplitude * sin(theta) * cos(phi)),
                            ScalarDriver::getConstantDriver(hAmplitude * sin(theta) * sin(phi)),
                            ScalarDriver::getConstantDriver(hAmplitude * cos(theta)));

                        mtj.clearLog();
                        mtj.setLayerExternalFieldDriver(
                            "all",
                            HDriver);
                        mtj.setLayerOerstedFieldDriver(
                            "all",
                            HoeDriver);
                        mtj.runSimulation(
                            time,
                            tStep, tWrite, false, false, false);

                        auto res = mtj.calculateVoltageSpinDiode(freq, power, tStart);
                        // Take last m value as well
                        auto resTuple = std::make_tuple(hAmplitude, freq, res["Vmix"],
                                                        mtj.layers[0].mag, mtj.layers[1].mag);
                        resAcc.push_back(resTuple);
                    }
                    freqIndx += 1;
                }
                return resAcc;
            }));
        }
        json finalRes;
        for (auto &result : threadResults)
        {
            for (const auto [h, freq, vmix, l1mag, l2mag] : result.get())
            {
                finalRes["H"].push_back(std::move(h));
                finalRes["frequencies"].push_back(std::move(freq));
                finalRes["Vmix"].push_back(std::move(vmix));
                finalRes["mags_1"].push_back({l1mag.x, l1mag.y, l1mag.z});
                finalRes["mags_2"].push_back({l2mag.x, l2mag.y, l2mag.z});
            }
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        spdlog::info("Total result retrieval time = {} [s]", std::chrono::duration_cast<std::chrono::seconds>(end - begin).count());
        finalRes["hsteps"] = std::move(hsteps);
        finalRes["fsteps"] = std::move(fsteps);

        return finalRes;
    }

public:
    Engine()
    {
        setupLogger();
    }
    void startServer(const char *IP = "0.0.0.0",
                     const int port = 8080)
    {
        spdlog::info("Starting server at: {}:{}", IP, port);
        httplib::Server svr;

        svr.Post("/queue",
                 [&](const httplib::Request &req,
                     httplib::Response &res,
                     const httplib::ContentReader &content_reader) {
                     std::string body;
                     content_reader([&](const char *data, size_t data_length) {
                         body.append(data, data_length);
                         return true;
                     });

                     boost::uuids::uuid uuid = boost::uuids::random_generator()();
                     const auto uuidStr = boost::lexical_cast<std::string>(uuid);
                     spdlog::info("New UUID created: {}", uuidStr);

                     auto parsed = json::parse(body);
                     parsed["uuid"] = uuidStr;
                     this->queue.enqueue(parsed);

                     // form response
                     json resp;
                     resp["uuid"] = uuidStr;
                     resp["message"] = "Accepted onto queue";
                     res.set_content(resp.dump(), "text/json");
                 });

        svr.Get("/task",
                [&](const httplib::Request &req, httplib::Response &res) {
                    auto uuid = req.get_param_value("uuid");
                    spdlog::info("Attempting to retrieve {}", uuid);
                    auto resp = saver.retrieveTask(uuid);
                    res.set_content(resp.dump(), "text/json");
                });

        svr.listen(IP, port);
    }

    void observeQueue()
    {
        spdlog::info("Starting queue watch");
        while (true)
        {
            auto task = this->queue.dequeue();
            spdlog::info("Popped the task off the queue: {}", task.at("uuid"));
            try
            {
                auto j = parser.parseJunction(task);
                const auto taskType = task.at("task").get<std::string>();
                json result;
                if (taskType == "pim")
                {
                    result = parseRunPIM(j, task);
                }
                else if (taskType == "vsd")
                {
                    result = parseRunVSD(j, task);
                }
                else
                {
                    throw std::runtime_error("Unknown task type passed! Aborting!");
                }
                spdlog::info("Attempting to save the task {} to the database", task.at("uuid"));
                saver.saveTaskResult(task, result);
            }
            catch (std::exception &e)
            {
                spdlog::error("Error occured {}! Abandoning task!", e.what());
                saver.saveFailedTaskResult(task, e);
            }
        }
    }
};