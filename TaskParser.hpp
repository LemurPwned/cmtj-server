#ifndef TASK_PARSER_H
#define TASK_PARSER_H

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <chrono>

#include <future>
#include <map>
#include <thread>

#include "./third/httplib.h"
#include "./third/junction.hpp"

#include "DatabaseHandler.hpp"
#include "QueueHandler.hpp"

#include <cmath>
#include <exception>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <tuple>

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
    else
    {
        spdlog::error("Invalid scalar driver object to set! Setting nothing!");
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

        Junction j;
        if (task.contains("Rp") && task.contains("Rap"))
        {
            j = Junction(layers, filename,
                         task.at("Rp").get<double>(),
                         task.at("Rap").get<double>());
        }
        else if (task.contains("Rx0") && task.contains("Ry0") &&
                 task.contains("AMR") && task.contains("AHE") &&
                 task.contains("SMR"))
        {
            j = Junction(layers, filename,
                         task.at("Rx0").get<std::vector<double>>(),
                         task.at("Ry0").get<std::vector<double>>(),
                         task.at("AMR").get<std::vector<double>>(),
                         task.at("AHE").get<std::vector<double>>(),
                         task.at("SMR").get<std::vector<double>>());
        }
        else
        {
            throw std::runtime_error("Failed to provide adequate MR parameters!");
        }

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

#endif