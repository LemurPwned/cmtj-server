#include <iostream>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <junction.hpp>

#include "QueueHandler.hpp"
#include "third/httplib.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class Engine
{
private:
    SafeQueue<json> queue;

    const std::string queueName = "taskQueue";
    void runSimulation(Junction j, double tStart, double tStop)
    {
    }

public:
    void startServer(const char *IP = "0.0.0.0",
                     const int port = 8080)
    {
        std::cout << "Starting server at: " << IP << ":" << port << std::endl;
        httplib::Server svr;

        svr.Get("/vsd", [](const httplib::Request &, httplib::Response &res) {
            res.set_content("Hello World!", "text/plain");
        });
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
        std::cout << "Starting observation queue" << std::endl;
        while (true)
        {
            auto task = this->queue.dequeue();
            std::cout << "Popped the task off the queue!" << std::endl;
            Layer l;
            parseLayer(task, l);
            std::cout << l.damping << " " << l.demagTensor[2].x << " " << l.demagTensor[2].y << " " << l.demagTensor[2].z << std::endl;
        }
    }

    void parseJsonString(std::string filename)
    {
        std::ifstream file(filename);
        json layerJson;
        file >> layerJson;
        Layer l;
        parseLayer(layerJson, l);
        std::cout << l.damping << " " << l.demagTensor[2].x << " " << l.demagTensor[2].y << " " << l.demagTensor[2].z << std::endl;
    }

    Junction parseIncomingJsonString(json junctionJson, std::string filename = "current.csv")
    {

        std::vector<Layer> layers = {};

        for (auto &layerDef : junctionJson["layers"].items())
        {
            Layer l;
            parseLayer(layerDef, l);
            layers.push_back(l);
        }

        Junction j(layers, filename, junctionJson.value("Rp", 100), junctionJson.value("Rap", 200));
        return j;
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