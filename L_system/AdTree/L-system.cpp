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

void Lsystem::readSkeleton(Skeleton* skel){
    std::cout << "nr. vertices of smooth skeleton: " << num_vertices(skel->get_smoothed_skeleton()) << std::endl;
    std::cout << "nr. vertices of simplified skeleton: " << num_vertices(skel->get_simplified_skeleton()) << std::endl;
    printLsystem();

    // get paths
    std::vector<Path> pathList;
    skel->get_graph_for_lsystem(pathList);

    std::cout << "number of paths: " << pathList.size() << std::endl;

    // for each path get its coordinates and generate a smooth curve
    for (std::size_t n_path = 0; n_path < pathList.size(); ++n_path) {
        if (n_path != 0){
            string_ += "]";
        }

        Path currentPath = pathList[n_path];
        std::vector<vec3> interpolatedPoints;

        std::cout << "\n---------- new subpath ----------" << std::endl;
        std::cout << "subpath length: " << currentPath.size() << std::endl;
        std::cout << "root subpath:   " << skel->get_simplified_skeleton()[currentPath[0]].cVert << std::endl;

        for (std::size_t n_node = 0; n_node < currentPath.size() - 1; ++n_node) {
            SGraphVertexDescriptor sourceV = currentPath[n_node];
            SGraphVertexDescriptor targetV = currentPath[n_node + 1];
            vec3 pSource = skel->get_simplified_skeleton()[sourceV].cVert;
            vec3 pTarget = skel->get_simplified_skeleton()[targetV].cVert;
            float branchlength = easy3d::distance(pSource, pTarget);

            std::cout << "source: " << skel->get_simplified_skeleton()[sourceV].cVert
                      << " --> target: " << skel->get_simplified_skeleton()[targetV].cVert
                      << std::endl;

            std::cout << "branch length: " << branchlength << std::endl;

            // compute the tangents
            vec3 tangentOfSorce;
            vec3 tangentOfTarget;
            // if the source vertex is the root
            if (sourceV == skel->get_simplified_skeleton()[sourceV].nParent) {
                tangentOfSorce = (pTarget - pSource).normalize();

                string_ += std::to_string(sourceV);
            }
            else
            {
                SGraphVertexDescriptor parentOfSource = skel->get_simplified_skeleton()[sourceV].nParent;
                tangentOfSorce = (pTarget - skel->get_simplified_skeleton()[parentOfSource].cVert).normalize();

                if (out_degree(sourceV, skel->get_simplified_skeleton()) > 2){
                    string_ += "[";
                }

            }
            // if the target vertex is leaf
            if ((out_degree(targetV, skel->get_simplified_skeleton()) == 1)
            && (targetV != skel->get_simplified_skeleton()[targetV].nParent)){
                tangentOfTarget = (pTarget - pSource).normalize();

                string_ += std::to_string(targetV);
                string_ += "]";
            }
            else
            {
                SGraphVertexDescriptor childOfTarget = currentPath[n_node + 2];
                tangentOfTarget = (skel->get_simplified_skeleton()[childOfTarget].cVert - pSource).normalize();

                string_ += std::to_string(targetV);
            }

            std::cout << "source tangent: " << tangentOfSorce
                      << " | target tangent: " << tangentOfTarget
                      << std::endl;

            tangentOfSorce *= branchlength;
            tangentOfTarget *= branchlength;
        }
    }
    std::cout << "writing L-system: done" << std::endl;
    printLsystem();
}
