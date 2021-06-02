//
// Created by noort on 02/06/2021.
//

#ifndef L_SYSTEM_LBRANCHGEN_H
#define L_SYSTEM_LBRANCHGEN_H


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

#include "AdTree/skeleton.h"
#include "3rd_party/nlohmann/json.hpp"
#include "AdTree/cylinder.h"
#include "AdTree/L-system.h"


class lbranchGen {

};


#endif //L_SYSTEM_LBRANCHGEN_H
