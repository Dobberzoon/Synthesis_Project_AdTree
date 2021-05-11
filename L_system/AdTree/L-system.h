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
    void Lsystem::writeMovement(SGraphVertexDescriptor startV,
                                SGraphVertexDescriptor nextV,
                                Skeleton *skel);

private:
    Graph graph_;
    std::string Lstring_;
    std::string axiom_;
    easy3d::vec3 zaxis_;

    easy3d::Vec<3, double> loc_;

    // todo: remove attributes we will never use

};

#endif //L_SYSTEM_L_SYSTEM_H
