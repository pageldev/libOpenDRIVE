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
        
        std::vector<std::shared_ptr<RoadGeometry>> geometries;
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
                geometries.push_back(std::make_shared<Line>(s0, x0, y0, hdg0, length));
            } else if( geometry_type == "spiral" ) {
                double curv_start = geometry_node.attribute("curvStart").as_double();
                double curv_end = geometry_node.attribute("curvEnd").as_double();
                geometries.push_back(std::make_shared<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end));
            } else if( geometry_type == "arc" ) {
                double curvature = geometry_node.attribute("curvature").as_double();
                geometries.push_back(std::make_shared<Arc>(s0, x0, y0, hdg0, length, curvature));
            } else if( geometry_type == "paramPoly3" ) {
                double aU = geometry_node.attribute("aU").as_double();
                double bU = geometry_node.attribute("bU").as_double();
                double cU = geometry_node.attribute("cU").as_double();
                double dU = geometry_node.attribute("dU").as_double();
                double aV = geometry_node.attribute("aV").as_double();
                double bV = geometry_node.attribute("bV").as_double();
                double cV = geometry_node.attribute("cV").as_double();
                double dV = geometry_node.attribute("dV").as_double();
                geometries.push_back(std::make_shared<ParamPoly3>(s0, x0, y0, hdg0, length, aU, bU, cU, dU, aV, bV, cV, dV));
            } else {
                std::cout << "Could not parse " << geometry_type << std::endl;
            }
        }
        std::shared_ptr<Road> road = std::make_shared<Road>(road_length, road_id, junction_id, geometries);
        this->roads.insert( std::pair<int, std::shared_ptr<Road>>(road->id, road) );

        /* read and sort lanesections by s */
        pugi::xpath_node_set lane_section__xpath_nodes = road_node.node().select_nodes(".//lanes//laneSection");
        std::vector<pugi::xml_node> lane_section_nodes;
        std::transform(lane_section__xpath_nodes.begin(), lane_section__xpath_nodes.end(), std::back_inserter(lane_section_nodes)
            , [](const pugi::xpath_node& a) { return a.node(); } ); 
        std::sort(lane_section_nodes.begin(), lane_section_nodes.end()
            , [](const pugi::xml_node& a, const pugi::xml_node& b){ 
                return a.attribute("s").as_double() < b.attribute("s").as_double(); } );

        std::vector<std::shared_ptr<LaneSection>> lane_sections;
        for( int idx = 0; idx < lane_section_nodes.size(); idx++ ) {
            double s0 = lane_section_nodes.at(idx).attribute("s").as_double();
            double lane_section_length = 0;
            if( (idx+1) < lane_section_nodes.size() ) {
                lane_section_length = lane_section_nodes.at(idx+1).attribute("s").as_double() - s0;
            } else {
                lane_section_length = road_length - s0;
            }
            std::shared_ptr<LaneSection> lane_section = std::make_shared<LaneSection>(s0, lane_section_length);
            lane_sections.push_back(lane_section);
            
            std::vector<std::shared_ptr<Lane>> lanes;
            for( pugi::xpath_node lane_node : lane_section_nodes.at(idx).select_nodes(".//lane") ) {
                int lane_id = lane_node.node().attribute("id").as_int();
                std::vector<std::shared_ptr<LaneWidth>> lane_widths;
                for( pugi::xpath_node lane_width_node : lane_node.node().select_nodes(".//width") ) {
                    double s_offset = lane_width_node.node().attribute("sOffset").as_double();
                    double a = lane_width_node.node().attribute("a").as_double();
                    double b = lane_width_node.node().attribute("b").as_double();
                    double c = lane_width_node.node().attribute("c").as_double();
                    double d = lane_width_node.node().attribute("d").as_double();
                    lane_widths.push_back(std::make_shared<LaneWidth>(s_offset, a, b, c, d));
                }
                lanes.push_back(std::make_shared<Lane>(lane_id, lane_widths));
            }
            lane_section->add_lane(lanes);
        }
        road->add_lanesection(lane_sections);
    }
}

void OpenDriveMap::export_as_obj(std::string out_file, double resolution)
{
    std::ofstream obj_file(out_file);
    int vert_idx = 1;
    for( int idx = 0; idx < roads.size(); idx++ ) {
        std::shared_ptr<Road> road = this->roads.at(idx);
        for( std::shared_ptr<LaneSection> lane_section : road->lane_sections ) {
            std::vector<int> sideA_idxs, sideB_idxs;
            auto outer_lanes = std::minmax_element(lane_section->lanes.begin(), lane_section->lanes.end()
                , [](const std::shared_ptr<Lane>& a, const std::shared_ptr<Lane>& b){ return a->id < b->id; } );
            for( int val = 0; val < int(lane_section->length/resolution); val++ ) {
                double s = val*resolution + lane_section->s0;
                std::pair<double, double> xy_pt_A = lane_section->id2lane.at((*outer_lanes.first)->id)->get_outer_border_pt(s);
                obj_file << "v " << xy_pt_A.first << " " << xy_pt_A.second << " 0.0" << std::endl;
                sideA_idxs.push_back(vert_idx++);
                std::pair<double, double> xy_pt_B = lane_section->id2lane.at((*outer_lanes.second)->id)->get_outer_border_pt(s);
                obj_file << "v " << xy_pt_B.first << " " << xy_pt_B.second << " 0.0" << std::endl;
                sideB_idxs.push_back(vert_idx++);
            }
            std::reverse(sideB_idxs.begin(), sideB_idxs.end());
            obj_file << "f ";
            for( int vert_idx : sideA_idxs ) obj_file << vert_idx << " ";
            for( int vert_idx : sideB_idxs ) obj_file << vert_idx << " ";
            obj_file << std::endl;
        }
    }
    obj_file.close();
}

void OpenDriveMap::export_as_json(std::string out_file, double resolution)
{
    Json::Value features;
    int feature_idx = 0;
    for( std::pair<int, std::shared_ptr<Road>> road : this->roads ) {
        for( std::shared_ptr<RoadGeometry> road_geometry : road.second->geometries ) {
            Json::Value coordinates;
            for( int sample_nr = 0; sample_nr < int(road_geometry->length/resolution); sample_nr++ ) {
                double s = road_geometry->s0 + sample_nr*resolution;
                std::pair<double, double> xy_pt = road_geometry->get_point(s);
                Json::Value position;
                position[0] = xy_pt.first;
                position[1] = xy_pt.second;
                coordinates[sample_nr] = position;
            }
            Json::Value geometry;
            geometry["type"] = "LineString";
            geometry["coordinates"] = coordinates;
            
            Json::Value properties;
            properties["road_id"] = road.second->id;
            properties["geometry_type"] = geometry_type2str.at(road_geometry->type);
            properties["length"] = road_geometry->length;
            
            Json::Value feature;
            feature["type"] = "Feature";
            feature["geometry"] = geometry;
            feature["properties"] = properties;

            features[feature_idx++] = feature;
        }
    }
    Json::Value feature_collection;
    feature_collection["type"] = "FeatureCollection";
    feature_collection["features"] = features;

    std::ofstream geojson_file(out_file);
    geojson_file << feature_collection;
    geojson_file.close();
}