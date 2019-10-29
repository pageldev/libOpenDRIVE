#include "OpenDriveMap.h"


OpenDriveMap::OpenDriveMap(std::string xodr_file) 
    : xodr_file(xodr_file)
{
    std::cout << this->xodr_file << std::endl;
    pugi::xml_document doc;
    if( !doc.load_file(xodr_file.c_str()) )
        throw std::runtime_error("Could not load xml");
    
    pugi::xpath_node_set roads = doc.select_nodes(".//road");
    for( pugi::xpath_node road_node : roads ) {
        double road_length = road_node.node().attribute("length").as_double();
        int road_id = road_node.node().attribute("id").as_int();
        int junction_id = road_node.node().attribute("junction").as_int();
        
        std::set<std::shared_ptr<RoadGeometry>, CmpRoadGeometry> geometries;
        pugi::xpath_node_set geometry_headers = road_node.node().select_nodes(".//planView//geometry");
        for( pugi::xpath_node geometry_hdr_node : geometry_headers ) {
            double s0 = geometry_hdr_node.node().attribute("s").as_double();
            double x0 = geometry_hdr_node.node().attribute("x").as_double();
            double y0 = geometry_hdr_node.node().attribute("y").as_double();
            double hdg0 = geometry_hdr_node.node().attribute("hdg").as_double();
            double length = geometry_hdr_node.node().attribute("length").as_double();
            pugi::xml_node geometry_node = geometry_hdr_node.node().first_child();
            std::string geometry_type = geometry_node.name();
            if( geometry_type == "line" ) {
                geometries.insert(std::make_shared<Line>(s0, x0, y0, hdg0, length));
            } else if( geometry_type == "spiral" ) {
                double curv_start = geometry_node.attribute("curvStart").as_double();
                double curv_end = geometry_node.attribute("curvEnd").as_double();
                geometries.insert(std::make_shared<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end));
            } else if( geometry_type == "arc" ) {
                double curvature = geometry_node.attribute("curvature").as_double();
                geometries.insert(std::make_shared<Arc>(s0, x0, y0, hdg0, length, curvature));
            } else if( geometry_type == "paramPoly3" ) {
                double aU = geometry_node.attribute("aU").as_double();
                double bU = geometry_node.attribute("bU").as_double();
                double cU = geometry_node.attribute("cU").as_double();
                double dU = geometry_node.attribute("dU").as_double();
                double aV = geometry_node.attribute("aV").as_double();
                double bV = geometry_node.attribute("bV").as_double();
                double cV = geometry_node.attribute("cV").as_double();
                double dV = geometry_node.attribute("dV").as_double();
                geometries.insert(std::make_shared<ParamPoly3>(s0, x0, y0, hdg0, length, aU, bU, cU, dU, aV, bV, cV, dV));
            } else {
                std::cout << "Could not parse " << geometry_type << std::endl;
            }
        }
        std::shared_ptr<Road> road = std::make_shared<Road>(road_length, road_id, junction_id, geometries);
        this->roads.push_back( road );

        /* read and sort lanesections by s */
        pugi::xpath_node_set lane_section__xpath_nodes = road_node.node().select_nodes(".//lanes//laneSection");
        std::vector<pugi::xml_node> lane_section_nodes;
        std::transform(lane_section__xpath_nodes.begin(), lane_section__xpath_nodes.end(), std::back_inserter(lane_section_nodes)
            , [](const pugi::xpath_node& a) { return a.node(); } ); 
        std::sort(lane_section_nodes.begin(), lane_section_nodes.end()
            , [](const pugi::xml_node& a, const pugi::xml_node& b){ 
                return a.attribute("s").as_double() < b.attribute("s").as_double(); } );

        // std::vector<std::shared_ptr<LaneSection>> lane_sections;
        std::set<std::shared_ptr<LaneSection>, CmpLaneSection> lane_sections;
        for( int idx = 0; idx < lane_section_nodes.size(); idx++ ) {
            double s0 = lane_section_nodes.at(idx).attribute("s").as_double();
            double lane_section_length = 0;
            if( (idx+1) < lane_section_nodes.size() ) {
                lane_section_length = lane_section_nodes.at(idx+1).attribute("s").as_double() - s0;
            } else {
                lane_section_length = road_length - s0;
            }
            std::shared_ptr<LaneSection> lane_section = std::make_shared<LaneSection>(s0, lane_section_length);
            lane_sections.insert(lane_section);
            
            std::set<std::shared_ptr<Lane>, CmpLane> lanes;
            for( pugi::xpath_node lane_node : lane_section_nodes.at(idx).select_nodes(".//lane") ) {
                int lane_id = lane_node.node().attribute("id").as_int();
                std::set<std::shared_ptr<LaneWidth>, CmpLaneWidth> lane_widths;
                for( pugi::xpath_node lane_width_node : lane_node.node().select_nodes(".//width") ) {
                    double s_offset = lane_width_node.node().attribute("sOffset").as_double();
                    double a = lane_width_node.node().attribute("a").as_double();
                    double b = lane_width_node.node().attribute("b").as_double();
                    double c = lane_width_node.node().attribute("c").as_double();
                    double d = lane_width_node.node().attribute("d").as_double();
                    lane_widths.insert(std::make_shared<LaneWidth>(s_offset, a, b, c, d));
                }
                lanes.insert(std::make_shared<Lane>(lane_id, lane_widths));
            }
            lane_section->add_lane(lanes);
        }
        road->add_lanesection(lane_sections);
    }
}


void OpenDriveMap::export_as_json(std::string out_file, double resolution)
{
    Point3D center_of_gravity {0.0, 0.0, 0.0};
    int nPoints = 0;
    for( std::shared_ptr<Road> road : this->roads ) {
        for( std::shared_ptr<RoadGeometry> geometry : road->geometries ) {
            center_of_gravity.x += geometry->x0;
            center_of_gravity.y += geometry->y0;
            nPoints++;
        }
    }
    center_of_gravity.x /= static_cast<double>(nPoints);
    center_of_gravity.y /= static_cast<double>(nPoints);
    
    Json::Value features;
    int feature_idx = 0;
    for( std::shared_ptr<Road> road : this->roads ) {
        for( std::shared_ptr<LaneSection> lane_section : road->lane_sections ) {
            for( std::shared_ptr<Lane> lane : lane_section->lanes ) {
                std::vector<Point3D> points;
                for( int sample_nr = 0; sample_nr < int(lane_section->length/resolution); sample_nr++ ) {
                    double s = lane_section->s0 + static_cast<double>(sample_nr)*resolution;
                    points.push_back(lane->get_outer_border_pt(s));
                }
                points.push_back( lane->get_outer_border_pt(lane_section->s0 + lane_section->length ) );
                Json::Value coordinates;
                std::vector<Point3D> reduced_points = rdp( points, resolution );           
                for( int idx = 0; idx < reduced_points.size(); idx++ ) {
                    Point3D pt = reduced_points.at(idx);
                    Json::Value position;
                    position[0] = pt.x - center_of_gravity.x;
                    position[1] = pt.y - center_of_gravity.y;
                    coordinates[idx] = position;
                }

                Json::Value geometry;
                geometry["type"] = "LineString";
                geometry["coordinates"] = coordinates;

                Json::Value properties;
                properties["road_id"] = road->id;

                Json::Value feature;
                feature["type"] = "Feature";
                feature["geometry"] = geometry;
                feature["properties"] = properties;

                features[feature_idx++] = feature;
            }
        }
    }
    Json::Value feature_collection;
    feature_collection["type"] = "FeatureCollection";
    feature_collection["features"] = features;

    std::ofstream geojson_file(out_file);
    geojson_file << feature_collection;
    geojson_file.close();
}