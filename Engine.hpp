#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <chrono>
#include <junction.hpp>

#include <map>

#include "QueueHandler.hpp"
#include "third/httplib.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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
        std::cout << "Parsing the layers" << std::endl;
        for (auto &layerDef : task["layers"])
        {
            Layer l;
            std::cout << layerDef << std::endl;
            this->parseLayer(layerDef, l);
            layers.push_back(l);
        }
        std::cout << "Finished parsing the layers" << std::endl;

        Junction j(layers, filename, task.value("Rp", 100), task.value("Rap", 200));

        std::cout << "Parsing the drivers" << std::endl;
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
        std::cout << "Finished parsing the drivers" << std::endl;
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
        std::cout << "Parsing the response" << std::endl;
        const double amplitude = vsdTask.at("vsdParams").at("amplitude").get<double>();
        const double phase = vsdTask.at("vsdParams").at("phase").get<double>();
        const int hsteps = vsdTask.at("vsdParams").at("hsteps").get<int>();

        const auto hstartVec = vsdTask.at("vsdParams").at("Hstart").get<std::vector<double>>();
        const auto hstepVec = vsdTask.at("vsdParams").at("Hstep").get<std::vector<double>>();

        CVector hstart(hstartVec);
        CVector hstep(hstepVec);

        const int fsteps = vsdTask.at("vsdParams").at("fsteps").get<int>();
        double fstart = vsdTask.at("vsdParams").at("frequencyStart").get<double>();
        double fstep = vsdTask.at("vsdParams").at("frequencyStep").get<double>();

        const double time = vsdTask.at("vsdParams").at("time").get<double>();
        const double tStep = vsdTask.at("vsdParams").at("tStep").get<double>();
        const double tWrite = vsdTask.at("vsdParams").at("tWrite").get<double>();
        const double tStart = vsdTask.at("vsdParams").at("tStart").get<double>();
        const double power = vsdTask.at("vsdParams").at("tStart").get<double>();
        runVSD(vsdTask.at("uuid").get<std::string>(), j,
               hstart, hstep, hsteps, amplitude, phase, fstart, fstep, fsteps, time, tStep, tWrite, tStart, power);
    }

    void runVSD(std::string uuid, Junction &j, CVector hstart, CVector hstep, int hsteps,
                double amplitude, double phase,
                double fstart, double fstep, int fsteps, double time, double tStep, double tWrite, double tStart, double power)
    {
        CVector hSweep = hstart;
        double fSweep = fstart;
        std::vector<double> frequencies, Vmix, Rpp;
        std::vector<int> fieldSteps;
        std::cout << "Calculating the VSD" << std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        for (int k = 0; k < fsteps; k++)
        {
            for (int i = 0; i < hsteps; i++)
            {
                j.clearLog();
                j.setLayerExternalFieldDriver(
                    "all",
                    AxialDriver(
                        ScalarDriver::getSineDriver(hSweep.x, amplitude, fSweep, phase),
                        ScalarDriver::getConstantDriver(hSweep.y),
                        ScalarDriver::getConstantDriver(hSweep.z)));
                j.runSimulation(
                    time,
                    tStep, tWrite, false, false, false);

                auto res = j.calculateVoltageSpinDiode(fSweep, power, tStart);
                frequencies.push_back(fSweep);
                fieldSteps.push_back(i);
                Vmix.push_back(res["Vmix"]);
                Rpp.push_back(res["Rpp"]);
                hSweep = hSweep + hstep;
            }
        }
        fSweep += fstep;
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Total simulation time = " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "[s]" << std::endl;
        json response = {
            {"frequencies", frequencies},
            {"fieldSteps", fieldSteps},
            {"Vmix", Vmix},
            {"Rpp", Rpp}};

        std::ofstream o(uuid + ".json");
        o << std::setw(4) << response << std::endl;
    }

public:
    void startServer(const char *IP = "0.0.0.0",
                     const int port = 8080)
    {
        std::cout << "Starting server at: " << IP << ":" << port << std::endl;
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
                     std::cout << "New UUID created: " << uuidStr << std::endl;

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
        std::cout << "Starting queue watch" << std::endl;
        while (true)
        {
            auto task = this->queue.dequeue();
            std::cout << "Popped the task off the queue!" << std::endl;
            std::cout << "Parsing the junction" << std::endl;
            auto j = parser.parseJunction(task);
            parseRunVSD(j, task);
        }
    }
};