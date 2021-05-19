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


class Lsystem
{
public:
    Lsystem();

    void printLsystem();

    void readSkeleton(Skeleton* skeleton);
    void traverse(SGraphVertexDescriptor prevV,
                  SGraphVertexDescriptor startV,
                  Skeleton *skeleton);

    std::tuple<double, double, double> moveToNext(SGraphVertexDescriptor startV,
                                                  SGraphVertexDescriptor nextV,
                                                  Skeleton *skeleton);
    void writeMovement(SGraphVertexDescriptor startV,
                                SGraphVertexDescriptor nextV,
                                Skeleton *skel,
                                int accuracy);

    double getZAngle(easy3d::vec3 vec);
    double getYAngle(easy3d::vec3 vec);

private:
    std::string Lstring_;
    std::string axiom_;
    bool degrees_;
};

#endif //L_SYSTEM_L_SYSTEM_H
