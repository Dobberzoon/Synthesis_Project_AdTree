//
// Created by noort on 06/05/2021.
//

#include "L-system.h"

#include "skeleton.h"
#include "cylinder.h"

#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/random.h>
#include <easy3d/core/principal_axes.h>
#include <3rd_party/tetgen/tetgen.h>

#include <iostream>
#include <algorithm>


using namespace boost;
using namespace easy3d;


Lsystem::Lsystem()
    {
        string_ = "";
        axiom_ = "";
        graph_.clear();
    }

void Lsystem::printLsystem() {
    std::cout << "printing L-system..." << std::endl;
    std::cout << "string: " << string_ << std::endl;
    std::cout << "axiom:  " << axiom_ << std::endl;
}

void Lsystem::readSkeleton(Skeleton* skeleton){
    std::cout << "nr. vertices of smooth skeleton: " << num_vertices(skeleton->get_smoothed_skeleton()) << std::endl;
    std::cout << "nr. vertices of simplified skeleton: " << num_vertices(skeleton->get_simplified_skeleton()) << std::endl;

    printLsystem();
}
