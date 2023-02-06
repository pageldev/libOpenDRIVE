#pragma once
#include "Math.hpp"
#include "Mesh.h"
#include "XmlNode.h"

#include <string>
#include <vector>

namespace odr
{
    struct RoadSignal : public XmlNode
    {
        RoadSignal(std::string road_id,
                   std::string id,
                   double s0,
                   double t0,
                   double z0,
                   double width,
                   double height,
                   double roll,
                   double pitch,
                   double yaw,
                   double h_offset,
                   double value,
                   std::string country,
                   std::string country_revision,
                   std::string type,
                   std::string subtype,
                   std::string text,
                   std::string unit,
                   std::string name,
                   bool dynamic
                   );

        static Mesh3D get_box(const double width,
                              const double length,
                              const double height);

        std::string road_id = "";
        std::string id = "";
        std::string name = "";
        std::string country = "";
        std::string country_revision = "";
        std::string type = "";
        std::string subtype = "";
        std::string text = "";
        std::string unit = "";

        double s0 = 0.0;
        double t0 = 0.0;
        double z0 = 0.0;
        double width = 0.0;
        double height = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        double hdg = 0.0;

        double value = 0.0;

        bool dynamic = false;
    };

}