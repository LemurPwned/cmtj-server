#include <iostream>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <junction.hpp>

#include <nlohmann/json.hpp>
#include <amqpcpp.h>
#include <amqpcpp/linux_tcp.h>

using json = nlohmann::json;

class Engine
{
private:
    boost::uuids::string_generator generator;

    const std::string fileRouter(const std::string &db)
    {
        boost::uuids::uuid uuid1 = generator("{01234567-89ab-cdef-0123-456789abcdef}");

        return boost::uuids::to_string(uuid1);
    }

    void runSimulation(Junction j, double tStart, double tStop)
    {
    }

public:

    void observeQueue(std::string topic){

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