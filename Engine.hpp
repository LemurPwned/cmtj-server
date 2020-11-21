#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
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

        for (auto &layerDef : task["layers"].items())
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
    void runDispersion(Junction &j, double frequency,
                       double Hstart, double Hstop, double Hstep,
                       double time, double tStep, double tWrite)
    {
        for (double h = Hstart; h < Hstop; h += Hstep)
        {
            j.clearLog();
            j.setLayerExternalFieldDriver(
                
            )
            j.runSimulation(
                time,
                tStep, tWrite, false, false, false);
        }
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

                     json resp;
                     auto parsed = json::parse(body);
                     this->queue.enqueue(parsed);
                     boost::uuids::uuid uuid = boost::uuids::random_generator()();
                     const auto uuidStr = boost::lexical_cast<std::string>(uuid);
                     std::cout << "New UUID created: " << uuidStr << std::endl;
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
            auto j = parser.parseJunction(task);
        }
    }
};