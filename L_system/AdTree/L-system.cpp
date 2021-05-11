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

// todo: zaxis_ not used, is this permanent?


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
    traverse(root, root, skel);

    std::cout << "writing L-system: done" << std::endl;
    printLsystem();

    // todo: write to json

    // todo: add starting position of root to json (direction not necessary)
}


void Lsystem::traverse(SGraphVertexDescriptor prevV,
                       SGraphVertexDescriptor startV,
                       Skeleton *skel){
    // check if node is not root
    // if not write the relative movement towards this node (startV)
    if (prevV != skel->get_simplified_skeleton()[prevV].nParent) {
        writeMovement(prevV, startV, skel);
    }
    // todo: does not write movement from root to first node, don't know why...
    // also writes index of node to the string, for debug only
    Lstring_ += "{" + std::to_string(startV) + "}";

    vec3 start_coords = skel->get_simplified_skeleton()[startV].cVert;
    SGraphVertexDescriptor nextV;
    std::vector<SGraphVertexDescriptor> slower_children;

    // skip if node is leaf
    if (!(out_degree(startV, skel->get_simplified_skeleton()) == 1)
        && (startV != skel->get_simplified_skeleton()[startV].nParent)) {

        /// find children of start node
        double maxR = -1;
        int isUsed = -1;
        std::pair<SGraphAdjacencyIterator, SGraphAdjacencyIterator> adjacencies =
                adjacent_vertices(startV, skel->get_simplified_skeleton());

        // shortest path search, copied from AdTree's method for making simplified_skeleton_
        for (SGraphAdjacencyIterator cIter = adjacencies.first; cIter != adjacencies.second; ++cIter) {
            // exclude parent
            if (*cIter != skel->get_simplified_skeleton()[startV].nParent) {
                SGraphEdgeDescriptor currentE = edge(startV, *cIter, skel->get_simplified_skeleton()).first;
                double radius = skel->get_simplified_skeleton()[currentE].nRadius;
                if (maxR < radius) {
                    maxR = radius;
                    if (isUsed > -1)
                        slower_children.push_back(nextV);
                    else
                        isUsed = 0;
                    nextV = *cIter;
                } else {
                    slower_children.push_back(*cIter);
                }
            }
        }

        /// start node has one child: straight segment
        if (out_degree(startV, skel->get_simplified_skeleton()) == 1) {
            traverse(startV, nextV, skel);
        }
        /// start node has multiple children: beginning of 2 or more branches
        else {
            // write the fastest child
            Lstring_ += "[";
            traverse(startV, nextV, skel);
            Lstring_ += "]";

            // also write all the other children
            for (int nChild = 0; nChild < slower_children.size(); ++nChild) {
                Lstring_ += "[";
                traverse(startV, slower_children[nChild], skel);
                Lstring_ += "]";
            }
        }
    }
}


std::tuple<double, double, double> Lsystem::moveToNext(SGraphVertexDescriptor startV,
                                                       SGraphVertexDescriptor nextV,
                                                       Skeleton *skel) {
    // end result: {angle around y-axis, angle anround z-axis, distance}
    std::tuple<double, double, double> movement{0, 0, 0};

    // get previous node (parent of start)
    SGraphVertexDescriptor prevV = skel->get_simplified_skeleton()[startV].nParent;

    // prevent it going wrong when start is [0 0 0] (is it's own parent, so vector between them is 0)
    if (startV != skel->get_simplified_skeleton()[startV].nParent) {
        // get coordinates of the two nodes
        vec3 coords_start = skel->get_simplified_skeleton()[startV].cVert;
        vec3 coords_next = skel->get_simplified_skeleton()[nextV].cVert;
        vec3 coords_prev = skel->get_simplified_skeleton()[prevV].cVert;

        // get distance between the two nodes
        float branch_length = easy3d::distance(coords_start, coords_next);

        /// compute vectors in coordinate system of startV
        // compute the vector (difference) between the two nodes
        vec3 to_target = (coords_next - coords_start);
        // compute the previous z-axis
        vec3 to_origin = (coords_start - coords_prev);

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
        double angle_z_orig = (2 * M_PI) - acos(dot(to_origin_xy, xaxis) / (length(to_origin_xy) * length(xaxis)));
        double angle_z_target = (2 * M_PI) - acos(dot(to_target_xy, xaxis) / (length(to_target_xy) * length(xaxis)));

        // todo: ugly fix...
        if (isnan(angle_z_orig)) {
            angle_z_orig = 0;
        }
        if (isnan(angle_z_target)) {
            angle_z_target = 0;
        }

        /// get rotation around Y
        // rotate vectors to XZ plane
        vec3 to_origin_xz = easy3d::mat3::rotation(zaxis, angle_z_orig) * to_origin;
        vec3 to_target_xz = easy3d::mat3::rotation(zaxis, angle_z_target) * to_target;

        // find angle between planar vectors & y-axis (acos = xaxis, y-axis = - 0.5 PI)
        double angle_y_orig = acos(dot(to_origin_xz, xaxis) / (length(to_origin_xz) * length(xaxis)));
        double angle_y_target = acos(dot(to_target_xz, xaxis) / (length(to_target_xz) * length(xaxis)));

        if (isnan(angle_y_orig)) {
            angle_y_orig = 0;
        }
        if (isnan(angle_y_target)) {
            angle_y_target = 0;
        }

        /// combine rotations
        double angle_diff_z = angle_z_orig - angle_z_target;
        double angle_diff_y = angle_y_orig - angle_y_target;

        // may still need these later, switched off for now.
        bool debug_print = false;

        if (debug_print) {
            std::cout << "\n---------- computing translation ----------" << std::endl;
            std::cout << "start node: (" << startV << ") " << coords_start
                      << " --> next: (" << nextV << ") " << coords_next
                      << " | previous: (" << prevV << ") " << coords_prev << std::endl;

            std::cout << "\nto origin proj XZ (y): " << to_origin_xz << std::endl;
            std::cout << "to target proj XZ (y): " << to_target_xz << std::endl;

            std::cout << "to_origin: " << to_origin << " , length: " << length(to_origin) << std::endl;
            std::cout << "to_target: " << to_target << " , length: " << length(to_target) << std::endl;

            std::cout << "\norigin angle z: " << angle_z_orig / (M_PI / 180) << std::endl;
            std::cout << "origin angle y: " << angle_y_orig / (M_PI / 180) << std::endl;
            std::cout << "target angle z: " << angle_z_target / (M_PI / 180) << std::endl;
            std::cout << "target angle y: " << angle_y_target / (M_PI / 180) << std::endl;
            std::cout << "diff. angle z: " << angle_diff_z / (M_PI / 180) << std::endl;
            std::cout << "diff. angle y: " << angle_diff_y / (M_PI / 180) << std::endl;

            std::cout << "\norigin angle y * xaxis: " << easy3d::mat3::rotation(0, angle_y_orig, 0) * xaxis
                      << std::endl;
            std::cout << "origin angle y * origin_xz(y): " << easy3d::mat3::rotation(0, angle_y_orig, 0) * to_origin_xz
                      << std::endl;
            std::cout << "origin angle z * origin_xy(z): " << easy3d::mat3::rotation(0, 0, angle_z_orig) * to_origin_xy
                      << std::endl;

            std::cout << "\nR * to_origin: "
                      << easy3d::mat3::rotation(0, angle_diff_y, -angle_z_orig + angle_diff_z, 123)
                         * easy3d::mat3::rotation(0, 0, angle_z_orig)
                         * to_origin << std::endl;
        }

        // makes sure angles close to 0 or 360 degrees get outputted as 0
        if (abs(2 * M_PI - angle_diff_y) > 0.001 && (abs(0 - angle_diff_y) > 0.001)) {
            std::get<0>(movement) = angle_diff_y;
        }
        if (abs(2 * M_PI - angle_diff_z) > 0.001 && (abs(0 - angle_diff_z) > 0.001)) {
            std::get<1>(movement) = angle_diff_z;
        }
        std::get<2>(movement) = branch_length;
    }
    return movement;
}


void Lsystem::writeMovement(SGraphVertexDescriptor startV,
                            SGraphVertexDescriptor nextV,
                            Skeleton *skel){
    // get relative movement between startV and nextV
    std::tuple<double, double, double> movement = moveToNext(startV, nextV, skel);
    int angle_y_deg = int(round(std::get<0>(movement) / (M_PI / 180)));
    int angle_z_deg = int(round(std::get<1>(movement) / (M_PI / 180)));
    double distance = std::get<2>(movement);

    /*std::cout << "\n---------- computing translation ----------" << std::endl;
    std::cout << "start node: (" << startV << ") " << skel->get_simplified_skeleton()[startV].cVert
              << " --> next: (" << nextV << ") " << skel->get_simplified_skeleton()[nextV].cVert
              << std::endl;

    std::cout << "angle: Y: " << angle_y_deg << std::endl;
    std::cout << "angle: Z: " << angle_z_deg << std::endl;
    std::cout << "disance:  " << distance << std::endl;*/

    // todo: do angles in a more neat way than just positive/negative... Can they be bigger than 360 deg?

    // todo: round angles and distance in a neater way (generalization?)

    /// write roll
    if (angle_y_deg > 0){
        Lstring_ += "+(" + std::to_string(angle_y_deg) + ")";
    }
    if (angle_y_deg < 0){
        Lstring_ += "-(" + std::to_string(abs(angle_y_deg)) + ")";
    }
    /// write rotation
    if (angle_z_deg > 0){
        Lstring_ += ">(" + std::to_string(angle_z_deg) + ")";
    }
    if (angle_z_deg < 0){
        Lstring_ += "<(" + std::to_string(abs(angle_z_deg)) + ")";
    }
    /// write forward
    if (distance > 0) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << distance;
        std::string dist_string = ss.str();
        Lstring_ += "F(" + dist_string + ")";
    }
}