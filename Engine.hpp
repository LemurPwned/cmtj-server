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

#include "QueueHandler.hpp"
#include "third/httplib.h"
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
}
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

    void parseRunVSD(Junction &j, json vsdTask)
    {
        spdlog::info("Parsing the VSD task");
        json subTaskDef = vsdTask.at("vsdParams");
        const double amplitude = subTaskDef.at("amplitude").get<double>();
        const double phase = subTaskDef.at("phase").get<double>();
        const int hsteps = subTaskDef.at("hsteps").get<int>();

        const auto hstartVec = subTaskDef.at("Hstart").get<std::vector<double>>();
        const auto hstepVec = subTaskDef.at("Hstep").get<std::vector<double>>();

        CVector hstart(hstartVec);
        CVector hstep(hstepVec);

        const int fsteps = subTaskDef.at("fsteps").get<int>();
        double fstart = subTaskDef.at("frequencyStart").get<double>();
        double fstep = subTaskDef.at("frequencyStep").get<double>();

        const double time = subTaskDef.at("time").get<double>();
        const double tStep = subTaskDef.at("tStep").get<double>();
        const double tWrite = subTaskDef.at("tWrite").get<double>();
        const double tStart = subTaskDef.at("tStart").get<double>();
        const double power = subTaskDef.at("tStart").get<double>();

        spdlog::info("FStart {}, FStep {}, #steps {}", fstart, fstep, fsteps);
        runVSD(vsdTask.at("uuid").get<std::string>(), // task UUID
               j,                                     // junction
               hstart,                                // start ext field vector
               hstep,                                 // step ext field vector
               hsteps,                                // no of steps
               amplitude,                             // amplitude of the excitation
               phase,                                 // phase of the excitation
               fstart,                                // start frequency
               fstep,                                 // step frequency
               fsteps,                                // number of freq steps
               time,
               tStep,
               tWrite,
               tStart,
               power);
    }

    typedef std::tuple<double, std::map<std::string, double>> fnRes;
    typedef std::tuple<int, double, double> trituple;
    void runVSD(std::string uuid, Junction &mtj, CVector hstart, CVector hstep, int hsteps,
                double amplitude, double phase,
                double fstart, double fstep, int fsteps, double time, double tStep, double tWrite, double tStart, double power)
    {

        double fSweep = fstart;
        // std::vector<double> frequencies, Vmix, Rpp;
        // std::vector<int> fieldSteps;
        spdlog::info("Calculating the VSD");
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // distribute the frequencies
        const int threadNum = std::thread::hardware_concurrency() - 1;
        spdlog::info("Using {} threads to distribute the task workload!", threadNum);

        std::vector<std::future<std::vector<trituple>>> threadResults;
        threadResults.reserve(threadNum);

        // const double threadSpacingFreq = (fstep * fsteps - fstart) / threadNum;
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
            // const int threadFreqSteps = t*
            spdlog::info("Launching thread {} with freq => {} to {}, load {}", t, threadMinFreq, threadMaxFreq, threadLoad);
            threadResults.emplace_back(std::async([=]() mutable {
                std::vector<trituple> resAcc;
                int freqIndx = 0;
                for (double freq = threadMinFreq; freqIndx < threadLoad; freq += fstep)
                {
                    CVector hSweep = hstart;
                    for (int i = 0; i < hsteps; i++)
                    {
                        mtj.clearLog();
                        mtj.setLayerExternalFieldDriver(
                            "all",
                            AxialDriver(
                                ScalarDriver::getSineDriver(hSweep.x, amplitude, freq, phase),
                                ScalarDriver::getConstantDriver(hSweep.y),
                                ScalarDriver::getConstantDriver(hSweep.z)));
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

        json response = {
            {"frequencies", finalRes["frequencies"]},
            {"fieldSteps", finalRes["fieldSteps"]},
            {"Vmix", finalRes["Vmix"]},
        };

        std::ofstream o(uuid + ".json");
        o << std::setw(4) << response << std::endl;
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        spdlog::info("Total result retrieval time = {} [s]", std::chrono::duration_cast<std::chrono::seconds>(end - begin).count());
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

        svr.listen(IP, port);
    }

    void observeQueue()
    {
        spdlog::info("Starting queue watch");
        while (true)
        {
            auto task = this->queue.dequeue();
            spdlog::info("Popped the task off the queue: {}", task.at("uuid"));
            auto j = parser.parseJunction(task);
            parseRunVSD(j, task);
        }
    }
};