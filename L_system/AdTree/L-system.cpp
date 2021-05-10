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
        zaxis_ = {0, 0, 1};
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


void Lsystem::moveToNext(SGraphVertexDescriptor startV, SGraphVertexDescriptor nextV, Skeleton *skel) {
    // get previous node (parent of start)
    // todo: this may break if start is the origin?
    SGraphVertexDescriptor prevV = skel->get_simplified_skeleton()[startV].nParent;

    // get coordinates of the two nodes
    vec3 coords_start = skel->get_simplified_skeleton()[startV].cVert;
    vec3 coords_next = skel->get_simplified_skeleton()[nextV].cVert;
    vec3 coords_prev = skel->get_simplified_skeleton()[prevV].cVert;

    // get distance between the two nodes
    float branch_length_origin = easy3d::distance(coords_start, coords_prev);
    float branch_length_target = easy3d::distance(coords_start, coords_next);

    /// compute vectors in coordinate system of startV
    // compute the vector (difference) between the two nodes
    vec3 to_target = (coords_next - coords_start);
    // compute the previous z-axis
    vec3 to_origin = (coords_start - coords_prev);

    // todo: catch nan values when to_target or to_origin are [0 0 1]

    vec3 xaxis = {1, 0, 0};
    vec3 yaxis = {0, 1, 0};
    vec3 zaxis = {0, 0, 1};

    /// get rotation around Z
    // project to xy plane
    vec3 to_origin_xy = {to_origin.x, to_origin.y, 0};
    vec3 to_target_xy = {to_target.x, to_target.y, 0};

    // find angle between planar vectors & x-axis
    // radians
    // todo: will this work in all quadrants or just the ++ one?
    // todo: could be more neat if this can be replaced by atan2()
    double angle_z_orig = acos(dot(to_origin_xy, xaxis) / (length(to_origin_xy) * length(xaxis)));
    double angle_z_target = acos(dot(to_target_xy, xaxis) / (length(to_target_xy) * length(xaxis)));

    /// get rotation around Y
    // rotate vectors to XZ plane
    vec3 to_origin_xz = easy3d::mat3::rotation(zaxis, -angle_z_orig) * to_origin;
    vec3 to_target_xz = easy3d::mat3::rotation(zaxis, -angle_z_target) * to_target;

    // find angle between planar vectors & y-axis (acos = xaxis, y-axis = + 3/2 PI)
    double angle_y_orig = (0.5*M_PI) - acos(dot(to_origin_xz, xaxis) / (length(to_origin_xz) * length(xaxis)));
    double angle_y_target = (0.5*M_PI) - acos(dot(to_target_xz, xaxis) / (length(to_target_xz) * length(xaxis)));

    /// combine rotations
    // respective rotations to rotate the original zaxis onto to_origin and to_target
    mat3 R_origin = easy3d::mat3::rotation(0, (angle_y_orig), (angle_z_orig), 123);
    mat3 R_target = easy3d::mat3::rotation(0, (angle_y_target), (angle_z_target), 123);

    double angle_diff_z = angle_z_target - angle_z_orig;
    double angle_diff_y = angle_y_target - angle_y_orig;

    mat3 R = easy3d::mat3::rotation(0, ((-1/2)*M_PI + angle_diff_y), angle_diff_z, 123);

    std::cout << "\n---------- computing translation ----------" << std::endl;
    std::cout << "start node: (" << startV << ") " << coords_start
              << " --> next: (" << nextV << ") " << coords_next
              << " | previous: (" << prevV << ") " << coords_prev << std::endl;

    std::cout << "\nto origin proj XZ (y): " << to_origin_xz << std::endl;
    std::cout << "to target proj XZ (y): " << to_target_xz << std::endl;

    std::cout << "to_origin: " << to_origin << " , length: " << length(to_origin) << std::endl;
    std::cout << "to_target: " << to_target << " , length: " << length(to_target) << std::endl;

    std::cout << "\norigin angle z: " << angle_z_orig / (M_PI/180) << std::endl;
    std::cout << "origin angle y: " << angle_y_orig / (M_PI/180) << std::endl;
    std::cout << "target angle z: " << angle_z_target / (M_PI/180) << std::endl;
    std::cout << "target angle y: " << angle_y_target / (M_PI/180) << std::endl;
    std::cout << "diff. angle z: " << (angle_z_target - angle_z_orig) / (M_PI/180) << std::endl;
    std::cout << "diff. angle y: " << (angle_y_target - angle_y_orig) / (M_PI/180) << std::endl;

    std::cout << "\nzaxis check to_origin: " << R_origin * zaxis << std::endl;
//    std::cout << "zaxis check to_origin V2: " << easy3d::mat3::rotation(zaxis, -angle_z_orig)
//    * easy3d::mat3::rotation(yaxis, (-1/2)*M_PI + angle_y_orig) * zaxis << std::endl;
    std::cout << "zaxis check to_target: " << R_target * zaxis << std::endl;

    std::cout << "\nR-orig:\n" << R_origin << std::endl;
    std::cout << "R-target:\n" << R_target << std::endl;


}