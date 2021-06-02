//
// Created by noort on 06/05/2021.
//

#ifndef L_SYSTEM_L_SYSTEM_H
#define L_SYSTEM_L_SYSTEM_H

#include <iostream>
#include <fstream>
#include <algorithm>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <3rd_party/kd_tree/Vector3D.h>
#include <3rd_party/kd_tree/KdTree.h>
#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/random.h>
#include <easy3d/core/principal_axes.h>
#include <3rd_party/tetgen/tetgen.h>

#include "skeleton.h"
#include "3rd_party/nlohmann/json.hpp"
#include "cylinder.h"
#include "generalisation/lbranchGen.h"


class Lsystem
{
public:
    Lsystem();

    Graph graph_lsys;
    // public for writing during generalisation
    std::string axiom;
    std::map<std::string, std::string> rules;

    /// get the root node index of the lsystem graph
    SGraphVertexDescriptor get_root(){return root_;};
    bool isDegrees(){return degrees_;};

    /// get a skeleton (from AdTree), convert it into an L-system, write it to output
    void readSkeleton(Skeleton* skeleton, bool deg);
    /// traverse all children of a node
    SGraphVertexDescriptor traverse(SGraphVertexDescriptor prevV,
                  SGraphVertexDescriptor startV,
                  Skeleton *skeleton);

    /// find the relative movement between two nodes
    std::tuple<double, double, double> moveToNext(SGraphVertexDescriptor startV,
                                                  SGraphVertexDescriptor nextV,
                                                  Skeleton *skeleton);

    /// get relative movement between two nodes, write it as L-string to the axiom
    void writeMovement(SGraphVertexDescriptor startV,
                                SGraphVertexDescriptor nextV,
                                Skeleton *skel,
                                int accuracy);

    /// get the angle between a vector and the x-axis around the z-axis
    double getZAngle(easy3d::vec3 vec);
    /// get the angle between a vector and the x-axis around the y-axis
    double getYAngle(easy3d::vec3 vec);

    /// write the L-system to an output (JSON, command line)
    void outputLsys(const std::string& out_type, const std::string& path);

    /// print the L-system to the command line
    void printLsystem();
    /// write the L-system to a custom JSON format
    void lsysToJson(const std::string &filenamel);
    /// write the L-system to a txt file
    void lsysToText(const std::string &filename);

    /// generalisation call
    void generalise();

private:
    std::string Lstring_;
    std::string axiom_;
    std::map<std::string, std::string> rules_;
    bool degrees_ = true;
    int rec_ = 0;
    SGraphVertexDescriptor root_;

    // default values
    float forward_ = 3;
    float rotation_ = 22.5;
    float roll_ = 20;

    //trunk values;
    easy3d::vec3 anchor_ = {0,0,0};
    float radius_ = 0.2;
};

#endif //L_SYSTEM_L_SYSTEM_H
