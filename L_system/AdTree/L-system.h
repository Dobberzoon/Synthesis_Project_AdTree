//
// Created by noort on 06/05/2021.
//

#ifndef L_SYSTEM_L_SYSTEM_H
#define L_SYSTEM_L_SYSTEM_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <3rd_party/kd_tree/Vector3D.h>
#include <3rd_party/kd_tree/KdTree.h>
#include <easy3d/core/types.h>

#include "skeleton.h"
#include "3rd_party/nlohmann/json.hpp"

class Lsystem
{
public:
    Lsystem();

    /// get a skeleton (from AdTree), convert it into an L-system, write it to output
    void readSkeleton(Skeleton* skeleton);
    /// traverse all children of a node
    void traverse(SGraphVertexDescriptor prevV,
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

    /// choose the output type
    enum outputFormat{
        OUT_COMMANDLINE,
        OUT_JSON_CUSTOM,
        OUT_TEXTFILE
    };
    /// write the L-system to an output (JSON, command line)
    void outputLsys(outputFormat out_type);

    /// print the L-system to the command line
    void printLsystem();
    /// write the L-system to a custom JSON format
    void lsysToJson(const std::string &filename,
                    double recursions,
                    double default_forward,
                    double default_rotation,
                    double default_roll);
    /// write the L-system to a txt file
    void lsysToText();

private:
    std::string Lstring_;
    std::string axiom_;
    bool degrees_;
};

#endif //L_SYSTEM_L_SYSTEM_H
