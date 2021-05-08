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
    void makePathsLsystem(Skeleton* skeleton, std::vector<Path> &pathList);
    void traverseLsystem(Skeleton* skeleton);
    void traverse(SGraphVertexDescriptor startV, Skeleton *skeleton);

private:
    Graph graph_;
    std::string string_;
    std::string axiom_;
};

#endif //L_SYSTEM_L_SYSTEM_H
