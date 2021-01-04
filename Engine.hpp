#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <chrono>
#include <junction.hpp>

#include <future>
#include <map>
#include <thread>

#include "./third/httplib.h"
#include "QueueHandler.hpp"
#include <cassert>
#include <leveldb/db.h>
#include <leveldb/write_batch.h>

#include <cmath>
#include <exception>
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

void setScalarDriver(const std::string &obj, Junction &j, const std::string &layerD, ScalarDriver driver)
{
    if (obj == "anisotropy")
    {
        j.setLayerAnisotropyDriver(layerD, driver);
    }
    else if (obj == "IEC")
    {
        j.setLayerIECDriver(layerD, driver);
    }
    else if (obj == "current")
    {
        j.setLayerIECDriver(layerD, driver);
    }
    else
    {
        spdlog::error("Invalid scalar driver object to set! Setting nothing!");
    }
}

class TaskDB
{
    leveldb::DB *db;
    leveldb::Options options;

public:
    TaskDB()
    {

        this->options.create_if_missing = true;
        leveldb::Status status = leveldb::DB::Open(options, "./testdb", &(this->db));
        if (!status.ok())
            spdlog::error("Status {}", status.ToString());
        assert(status.ok());
    }

    void saveFailedTaskResult(json input, std::exception &e)
    {
        spdlog::info("Saving failed task info...");
        std::string input_string = input.dump();
        std::string key = input["uuid"];

        json res;
        res["message"] = e.what();
        res["code"] = 304;

        leveldb::WriteBatch batch;
        batch.Put(key + "-input", input_string);
        batch.Put(key + "-output", res.dump());
        leveldb::Status s = this->db->Write(leveldb::WriteOptions(), &batch);
        if (!s.ok())
        {
            spdlog::error("Error {}", s.ToString());
        }
        else
        {
            spdlog::info("Status: {}", s.ToString());
        }
    }

    void saveTaskResult(json input, json output)
    {
        std::string input_string = input.dump();
        std::string output_string = output.dump();
        std::string key = input["uuid"];
        leveldb::WriteBatch batch;
        batch.Put(key + "-input", input_string);
        batch.Put(key + "-output", output_string);
        leveldb::Status s = this->db->Write(leveldb::WriteOptions(), &batch);

        if (!s.ok())
        {
            spdlog::error("Error {}", s.ToString());
        }
        else
        {
            spdlog::info("Status: {}", s.ToString());
        }
    }

    json retrieveTask(std::string uuid)
    {
        json response;
        std::string value;
        leveldb::Status s = this->db->Get(leveldb::ReadOptions(), uuid + "-output", &value);
        response["status"] = s.ToString();
        if (!s.ok())
        {
            response["code"] = 404;
            response["result"] = "Fetching failed";
        }
        else
        {
            response["code"] = 200;
            response["result"] = json::parse(value);
        }
        return response;
    }

    ~TaskDB()
    {
        delete this->db;
    }
};

class TaskParser
{
public:
    std::map<std::string, double> parseParamMap(json task)
    {
        std::map<std::string, double> params;
        for (auto &f : task.at("vsdParams").items())
        {
            params[f.key()] = f.value();
        }

        return params;
    }
    Junction parseJunction(json task, std::string filename = "current.csv")
    {

        std::vector<Layer> layers = {};
        for (auto &layerDef : task["layers"])
        {
            Layer l;
            this->parseLayer(layerDef, l);
            layers.push_back(l);
        }

        Junction j(layers, filename, task.value("Rp", 100), task.value("Rap", 200));

        for (auto driverDef : task.at("drivers"))
        {
            const auto layerD = driverDef.at("layer"); // top, free, bottom
            const auto obj = driverDef.at("object");

            if (obj == "externalField")
            {
                auto dr = AxialDriver(
                    this->parseScalarDriver(driverDef["x"]),
                    this->parseScalarDriver(driverDef["y"]),
                    this->parseScalarDriver(driverDef["z"]));
                j.setLayerExternalFieldDriver(layerD, dr);
            }
            else
            {
                const auto dr = parseScalarDriver(driverDef);
                setScalarDriver(obj, j, layerD, dr);
            }
        }
        spdlog::info("Finished parsing the driver options");
        return j;
    }

    ScalarDriver parseScalarDriver(json driverDef)
    {
        auto driverType = driverDef.at("subtype").get<std::string>();
        if (driverType == "constant")
        {
            return ScalarDriver::getConstantDriver(driverDef.at("constantValue"));
        }
        else if (driverType == "pulse")
        {
            return ScalarDriver::getPulseDriver(driverDef.at("constantValue"),
                                                driverDef.at("amplitude"),
                                                driverDef.at("period"),
                                                driverDef.at("cycle"));
        }
        else if (driverType == "sine")
        {
            return ScalarDriver::getSineDriver(
                driverDef.at("constantValue"),
                driverDef.at("amplitude"),
                driverDef.at("frequency"),
                driverDef.at("phase"));
        }
        else if (driverType == "pulse")
        {
            return ScalarDriver::getStepDriver(
                driverDef.at("constantValue"),
                driverDef.at("amplitude"),
                driverDef.at("timeStart"),
                driverDef.at("timeStop"));
        }
        else
        {
            throw std::runtime_error("Invalid scalar driver type!");
        }
    }

    void parseLayer(json layerDef, Layer &l)
    {

        layerDef.at("id").get_to(l.id);
        layerDef.at("Ms").get_to(l.Ms);
        layerDef.at("thickness").get_to(l.thickness);
        layerDef.at("Ms").get_to(l.Ms);
        layerDef.at("cellSurface").get_to(l.cellSurface);
        layerDef.at("includeSTT").get_to(l.includeSTT);
        layerDef.at("spinPolarisation").get_to(l.spinPolarisation);
        layerDef.at("beta").get_to(l.beta);
        layerDef.at("damping").get_to(l.damping);
        layerDef.at("SlonczewskiSpacerLayerParameter").get_to(l.SlonczewskiSpacerLayerParameter);

        const auto magVec = layerDef.at("mag").get<std::vector<double>>();
        const auto anisVec = layerDef.at("anis").get<std::vector<double>>();
        const auto demagVec = layerDef.at("demagTensor").get<std::vector<std::vector<double>>>();
        const auto dipoleVec = layerDef.at("dipoleTensor").get<std::vector<std::vector<double>>>();

        l.mag = CVector(magVec);
        l.anis = CVector(anisVec);
        l.demagTensor = std::vector<CVector>{
            CVector(demagVec[0]),
            CVector(demagVec[1]),
            CVector(demagVec[2]),
        };
        l.dipoleTensor = std::vector<CVector>{
            CVector(dipoleVec[0]),
            CVector(dipoleVec[1]),
            CVector(dipoleVec[2]),
        };
    }
};

class Engine
{
private:
    SafeQueue<json> queue;
    TaskParser parser;
    TaskDB saver;

    json parseRunVSD(Junction &j, json vsdTask)
    {
        spdlog::info("Parsing the VSD task");
        json subTaskDef = vsdTask.at("vsdParams");
        const double phase = subTaskDef.at("phase").get<double>();

        const auto hMin = subTaskDef.at("Hmin").get<double>();
        const auto hMax = subTaskDef.at("Hmax").get<double>();
        const int hsteps = subTaskDef.at("Hsteps").get<int>();

        const auto Hoe = subTaskDef.at("HOe").get<double>();
        const auto HoeDir = subTaskDef.at("HOedir").get<std::vector<double>>();

        const auto theta = subTaskDef.at("theta").get<double>();
        const auto phi = subTaskDef.at("phi").get<double>();

        const int fsteps = subTaskDef.at("fsteps").get<int>();
        double fstart = subTaskDef.at("fmin").get<double>();
        double fstop = subTaskDef.at("fmax").get<double>();

        const double time = subTaskDef.at("time").get<double>();
        const double tStep = subTaskDef.at("tStep").get<double>();
        const double tWrite = subTaskDef.at("tWrite").get<double>();
        const double tStart = subTaskDef.at("tStart").get<double>();
        const double power = subTaskDef.at("tStart").get<double>();

        spdlog::info("Fmin {}, Fmax {}, #steps {}", fstart, fstop, fsteps);
        spdlog::info("Hmin {}, Hmax {}, #steps {}", hMin, hMax, hsteps);
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

    typedef std::tuple<double, std::map<std::string, double>> fnRes;
    typedef std::tuple<int, double, double> trituple;
    json runVSD(std::string uuid, Junction &mtj, double Hoe, CVector HoeDir, double hMin,
                double hMax, double theta, double phi, int hsteps,
                double phase,
                double fstart, double fstop, int fsteps, double time,
                double tStep, double tWrite, double tStart, double power, bool save = false)
    {

        double fstep = (fstop - fstart) / fsteps;
        double hstep = (hMax - hMin) / hsteps;
        spdlog::info("Calculating the VSD");
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // distribute the frequencies
        const int threadNum = std::thread::hardware_concurrency() - 1;
        spdlog::info("Using {} threads to distribute the task workload!", threadNum);

        std::vector<std::future<std::vector<trituple>>> threadResults;
        threadResults.reserve(threadNum);

        const int minThreadLoad = (int)fsteps / threadNum;
        int extraThreadWorkload = fsteps % threadNum;
        spdlog::info("Min subtask load {} per thread", minThreadLoad);
        for (int t = 0; t < threadNum; t++)
        {
            int threadLoad = minThreadLoad;
            if (extraThreadWorkload)
            {
                threadLoad += 1;
                extraThreadWorkload--;
            }
            const double threadMinFreq = fstart + t * fstep;
            const double threadMaxFreq = fstart + (t + 1) * fstep;

            spdlog::info("Launching thread {} with freq => {} to {}, load {}", t, threadMinFreq, threadMaxFreq, threadLoad);
            threadResults.emplace_back(std::async([=]() mutable {
                std::vector<trituple> resAcc;
                int freqIndx = 0;
                for (double freq = threadMinFreq; freqIndx < threadLoad; freq += fstep)
                {
                    for (int i = 0; i < hsteps; i++)
                    {
                        const double hAmplitude = hMin + i * hstep;

                        const AxialDriver HDriver(
                            ScalarDriver::getConstantDriver(hAmplitude * sin(theta) * cos(phi)),
                            ScalarDriver::getConstantDriver(hAmplitude * sin(theta) * sin(phi)),
                            ScalarDriver::getConstantDriver(hAmplitude * cos(theta)));

                        const AxialDriver HoeDriver(
                            ScalarDriver::getSineDriver(0, HoeDir.x * Hoe, freq, phase),
                            ScalarDriver::getSineDriver(0, HoeDir.y * Hoe, freq, phase),
                            ScalarDriver::getSineDriver(0, HoeDir.z * Hoe, freq, phase));

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
                        auto resTuple = std::make_tuple(i, freq, res["Vmix"]);
                        resAcc.push_back(resTuple);
                    }
                    freqIndx += 1;
                }
                return resAcc;
            }));
        }
        std::map<std::string, std::vector<double>> finalRes;
        for (auto &result : threadResults)
        {
            for (const auto [fieldIndx, freq, vmix] : result.get())
            {
                finalRes["fieldSteps"].push_back(fieldIndx);
                finalRes["frequencies"].push_back(freq);
                finalRes["Vmix"].push_back(vmix);
            }
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        spdlog::info("Total result retrieval time = {} [s]", std::chrono::duration_cast<std::chrono::seconds>(end - begin).count());
        json response = {
            {"frequencies", finalRes["frequencies"]},
            {"fieldSteps", finalRes["fieldSteps"]},
            {"Vmix", finalRes["Vmix"]},
        };

        if (save)
        {
            std::ofstream o("./tmp/" + uuid + ".json");
            o << std::setw(4) << response << std::endl;
            std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
            spdlog::info("Total result save time = {} [s]", std::chrono::duration_cast<std::chrono::seconds>(end2 - end).count());
        }

        return response;
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

                auto result = parseRunVSD(j, task);
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