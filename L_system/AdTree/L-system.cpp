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
        Lstring_ = "";
        axiom_ = "";
        graph_.clear();

        plane_.set_row(0, easy3d::Vec<3, double>{1,0,0});
        plane_.set_row(1, easy3d::Vec<3, double>{0,1,0});
        plane_.set_row(2, easy3d::Vec<3, double>{0,0,1});
    }

void Lsystem::printLsystem() {
    std::cout << "printing L-system..." << std::endl;
    std::cout << "string: " << Lstring_ << std::endl;
    std::cout << "axiom:  " << axiom_ << std::endl;
}


void Lsystem::readSkeleton(Skeleton *skel) {
    std::cout << "nr. vertices of simplified skeleton: " << num_vertices(skel->get_simplified_skeleton()) << std::endl;
    printLsystem();

    SGraphVertexDescriptor root = skel->get_root();
    vec3 coords_root = skel->get_simplified_skeleton()[root].cVert;
    loc_ = {coords_root.x, coords_root.y, coords_root.z};
    traverse(root, skel);

    std::cout << "writing L-system: done" << std::endl;
    printLsystem();

    // todo: write to json

    // todo: add starting position of root to json (direction not necessary)
}


void Lsystem::traverse(SGraphVertexDescriptor startV, Skeleton *skel){
    // todo: change this to note the movement needed to reach the next node
    vec3 start_coords = skel->get_simplified_skeleton()[startV].cVert;
    Lstring_ += std::to_string(startV);
    //

//    std::cout << "\n---------- new branch ----------" << std::endl;
//    std::cout << "starting node: " << startV << std::endl;

    SGraphVertexDescriptor nextV;
    std::vector<SGraphVertexDescriptor> slower_children;

    /// node is leaf
    if ((out_degree(startV, skel->get_simplified_skeleton()) == 1)
        && (startV != skel->get_simplified_skeleton()[startV].nParent)) {

//        std::cout << "leaf found: " << startV << std::endl;
    }
    else{
        /// find children of start node
        double maxR = -1;
        int isUsed = -1;

        std::pair<SGraphAdjacencyIterator, SGraphAdjacencyIterator> adjacencies =
                adjacent_vertices(startV, skel->get_simplified_skeleton());

        for (SGraphAdjacencyIterator cIter = adjacencies.first; cIter != adjacencies.second; ++cIter)
        {
            // exclude parent
            if (*cIter != skel->get_simplified_skeleton()[startV].nParent)
            {
//                std::cout << "child found: " << *cIter << std::endl;
                SGraphEdgeDescriptor currentE = edge(startV, *cIter, skel->get_simplified_skeleton()).first;
                double radius = skel->get_simplified_skeleton()[currentE].nRadius;
                if (maxR < radius)
                {
                    maxR = radius;
                    if (isUsed > -1)
                        slower_children.push_back(nextV);
                    else
                        isUsed = 0;
                    nextV = *cIter;
                }
                else
                    slower_children.push_back(*cIter);
            }
        }


//        std::cout << "fastest child: " << nextV << std::endl;
//        std::cout << "slower children: " ;
//
//        for (int nChild = 0; nChild < slower_children.size(); ++nChild){
//            std::cout << slower_children[nChild] << " ";
//        }

        /// start node has one child: straight segment
        if (out_degree(startV, skel->get_simplified_skeleton()) == 1){
            traverse(nextV, skel);

            moveToNext(startV, nextV, skel);
        }
        /// start node has multiple children: beginning of 2 or more branches
        else{
            Lstring_ += "[";
            traverse(nextV, skel);
            Lstring_ += "]";

            moveToNext(startV, nextV, skel);

            for (int nChild = 0; nChild < slower_children.size(); ++nChild){
                Lstring_ += "[";
                traverse(slower_children[nChild], skel);
                Lstring_ += "]";

                moveToNext(startV, nextV, skel);
            }
        }
    }
}


void Lsystem::moveToNext(SGraphVertexDescriptor startV, SGraphVertexDescriptor nextV, Skeleton *skel){
    // get coordinates of the two nodes
    vec3 coords_start = skel->get_simplified_skeleton()[startV].cVert;
    vec3 coords_next = skel->get_simplified_skeleton()[nextV].cVert;
    // get distance between the two nodes
    float branch_length = easy3d::distance(coords_start, coords_next);

    std::cout << "\n---------- computing translation ----------" << std::endl;
    std::cout << "start node: (" << startV << ") " << coords_start
              << " --> next: (" << nextV << ") " << coords_next << std::endl;

    // compute the tangent
    vec3 tangent;
    tangent = (coords_next - coords_start).normalize();
    vec3 zaxis = {0, 0, 1};

    double angle_rad = asin(norm(cross(zaxis, tangent)));
//    double angle_rad = angle_deg * M_PI/180;
    double angle_deg = angle_rad / (M_PI/180);
    mat3 rot;
    rot = rot.rotation(cross(zaxis, tangent), angle_rad);

    std::cout << "length tangent: " << round(length(tangent)) << std::endl;
    std::cout << "tangent:         " << tangent << std::endl;
    std::cout << "rotation result: " << rot * zaxis << std::endl;
    std::cout << "ange rad: " << angle_rad << " , angle degrees: " << round(angle_deg) << std::endl;
    std::cout << "rotation matrix:\n" << rot << std::endl;

}

/// step forward ///
void Lsystem::stepForward(double distance){
    loc_ += distance * plane_.row(2);
}

/// rotate ///
void Lsystem::rotatePlane(double angle){
    angle = angle * M_PI/180;

    easy3d::Vec<3, double> uAxis = plane_.row(0);
    easy3d::Vec<3, double> vAxis = plane_.row(2);

    easy3d::Vec<3, double> uAxisT = uAxis*cos(angle) - vAxis*sin(angle);
    easy3d::Vec<3, double> vAxisT = uAxis*sin(angle) + vAxis*cos(angle);

    uAxisT.normalize();
    vAxisT.normalize();

    plane_.set_row(0, uAxisT);
    plane_.set_row(2, vAxisT);
}

/// roll ///
void Lsystem::rollPlane(double angle){
    angle = angle * M_PI/180;

    easy3d::Vec<3, double> uAxis = plane_.row(0);
    easy3d::Vec<3, double> wAxis = plane_.row(1);

    easy3d::Vec<3, double> uAxisT = uAxis*cos(angle) - wAxis*sin(angle);
    easy3d::Vec<3, double> wAxisT = uAxis*sin(angle) + wAxis*cos(angle);

    uAxisT.normalize();
    wAxisT.normalize();

    plane_.set_row(0, uAxisT);
    plane_.set_row(1, wAxisT);
}