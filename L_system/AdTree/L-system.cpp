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
        plane_ = mat3(1);
        degrees_ = false;
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
    degrees_ = true;
    traverse(root, root, skel);

    std::cout << "writing L-system: done" << std::endl;
    printLsystem();

    // todo: write to json

    // todo: add starting position of root to json (direction not necessary)
}


void Lsystem::traverse(SGraphVertexDescriptor prevV,
                       SGraphVertexDescriptor startV,
                       Skeleton *skel){
    // write movement from prevV to nextV to Lstring
    // will not write when prevV is a leaf or the root (preventing doubles and the root writing to itself)
    writeMovement(prevV, startV, skel);

    // also writes index of node to the string, for debug only
//    Lstring_ += "{" + std::to_string(startV) + "}";

    vec3 start_coords = skel->get_simplified_skeleton()[startV].cVert;
    SGraphVertexDescriptor nextV;
    std::vector<SGraphVertexDescriptor> slower_children;

    // skip if node is leaf
    if (!((out_degree(startV, skel->get_simplified_skeleton()) == 1)
        && (startV != skel->get_simplified_skeleton()[startV].nParent))) {

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
    if (nextV != skel->get_simplified_skeleton()[nextV].nParent) {
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
        double angle_z_orig = getZAngle(to_origin_xy);
        double angle_z_target = getZAngle(to_target_xy);

        // todo: make 360 --> 0

        // todo: 360+ or -360-?

        // todo: ugly fix...
        if (isnan(angle_z_orig)) {
            angle_z_orig = 0;
        }
        if (isnan(angle_z_target)) {
            angle_z_target = 0;
        }

        /// get rotation around Y
        // rotate vectors to XZ plane
        vec3 to_origin_xz = easy3d::mat3::rotation(zaxis, -angle_z_orig) * to_origin;
        vec3 to_target_xz = easy3d::mat3::rotation(zaxis, -angle_z_target) * to_target;

        // find angle between planar vectors & y-axis (acos = xaxis, y-axis = - 0.5 PI)
        double angle_y_orig = getYAngle(to_origin_xz);
        double angle_y_target = getYAngle(to_target_xz);

        if (isnan(angle_y_orig)) {
            angle_y_orig = 0;
        }
        if (isnan(angle_y_target)) {
            angle_y_target = 0;
        }

        /// combine rotations
        double angle_diff_z = angle_z_target - angle_z_orig;
        double angle_diff_y = angle_y_target - angle_y_orig;

        // may still need these later, switched off for now.
        bool debug_print = true;

        if (debug_print) {

            std::cout << "\n---------- computing translation ----------" << std::endl;
            std::cout << "start node: (" << startV << ") " << coords_start
                      << " --> next: (" << nextV << ") " << coords_next
                      << " | previous: (" << prevV << ") " << coords_prev << std::endl;

//            std::cout << "\nto origin proj XZ (y): " << to_origin_xz << std::endl;
//            std::cout << "to target proj XZ (y): " << to_target_xz << std::endl;

            std::cout << "to_origin: " << to_origin << " , length: " << length(to_origin) << std::endl;
            std::cout << "to_target: " << to_target << " , length: " << length(to_target) << std::endl;
            if (length(to_origin) == 0) {
                to_origin = {1, 0, 0};
            }
            vec3 next_vec = easy3d::mat3::rotation(0, angle_diff_y, angle_z_orig + angle_diff_z, 123)
                            * easy3d::mat3::rotation(0, 0, -angle_z_orig)
                            * to_origin.normalize() * length(to_target);
            std::cout << "R * to_origin normalized * length target: " << next_vec << std::endl;
            std::cout << "next node calculated: " << coords_start + next_vec << std::endl;

            std::cout << "\norigin angle z: " << angle_z_orig / (M_PI / 180) << std::endl;
            std::cout << "origin angle y: " << angle_y_orig / (M_PI / 180) << std::endl;
            std::cout << "target angle z: " << angle_z_target / (M_PI / 180) << std::endl;
            std::cout << "target angle y: " << angle_y_target / (M_PI / 180) << std::endl;
            std::cout << "diff. angle z: " << angle_diff_z / (M_PI / 180) << std::endl;
            std::cout << "diff. angle y: " << angle_diff_y / (M_PI / 180) << std::endl;

            /*std::cout << "\n---------- turtle test ----------" << std::endl;
            std::cout << "previous loc: " << loc_ << std::endl;
            std::cout << "previous plane: " << plane_ << std::endl;

            rotatePlane(angle_diff_y);
            std::cout << "rotate plane: " << plane_ <<  std::endl;

            rollPlane(angle_diff_z);
            std::cout << "roll plane: " << plane_ <<  std::endl;

            stepForward(branch_length);
            std::cout << "next loc: " << loc_ << std::endl;*/
        }

        // make sure angles very close to 0 or 360 degrees get outputted as 0
        if (abs(2 * M_PI - angle_diff_y) > 0.0001 && (abs(0 - angle_diff_y) > 0.0001)) {
            std::get<0>(movement) = angle_diff_y;
        }
        if (abs(2 * M_PI - angle_diff_z) > 0.0001 && (abs(0 - angle_diff_z) > 0.0001)) {
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
    // angles are rounded to int
    double angle_y = std::get<0>(movement);
    double angle_z = std::get<1>(movement);
    // option to output as degrees instead of radians
    if (degrees_){
        angle_y = std::get<0>(movement) / (M_PI / 180);
        angle_z = std::get<1>(movement) / (M_PI / 180);
    }
    double distance = std::get<2>(movement);

    /*std::cout << "\n---------- computing translation ----------" << std::endl;
    std::cout << "start node: (" << startV << ") " << skel->get_simplified_skeleton()[startV].cVert
              << " --> next: (" << nextV << ") " << skel->get_simplified_skeleton()[nextV].cVert
              << std::endl;

    std::cout << "angle: Y: " << angle_y_deg << std::endl;
    std::cout << "angle: Z: " << angle_z_deg << std::endl;
    std::cout << "disance:  " << distance << std::endl;*/

    // todo: round angles and distance in a neater way (generalization?)

    /// write rotation
    if (angle_y > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << angle_y;
        std::string angle_y_string = ss.str();
        Lstring_ += "+(" + angle_y_string + ")";
    }
    if (angle_y < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << abs(angle_y);
        std::string angle_y_string = ss.str();
        Lstring_ += "-(" + angle_y_string + ")";
    }
    /// write roll
    if (angle_z > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << angle_z;
        std::string angle_z_string = ss.str();
        Lstring_ += ">(" + angle_z_string + ")";
    }
    if (angle_z < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << abs(angle_z);
        std::string angle_z_string = ss.str();
        Lstring_ += "<(" + angle_z_string + ")";
    }
    /// write forward
    if (distance > 0) {
        // round distance to 2 decimals
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << distance;
        std::string dist_string = ss.str();
        Lstring_ += "F(" + dist_string + ")";
    }
}


double Lsystem::getZAngle(vec3 vec){
    vec3 xaxis = {1, 0, 0};
    double angle_z = 0;

    // angle is dependent on what side of the x-axis (XY plane) the vector is
    if (vec.y < 0){
        angle_z = - acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }
    else{
        angle_z = acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }

    return angle_z;
}

double Lsystem::getYAngle(vec3 vec){
    vec3 xaxis = {1, 0, 0};
    double angle_y = 0;

    // angle is dependent on what side of the x-axis (XZ plane) the vector is
    if (vec.z < 0) {
        angle_y = acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }
    else{
        angle_y = (2 * M_PI) - acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }

    return angle_y;
}

/// step forward ///
void Lsystem::stepForward(double distance){
    loc_ = loc_ + (plane_ * vec3(1, 0, 0) * distance);
}

/// rotate ///
void Lsystem::rotatePlane(double angle){
//    angle = angle * M_PI/180;

    /// 1: roll to XZ plane
    // project current z-axis onto XY plane;
    vec3 xAxis = plane_ * vec3(1, 0, 0);
    vec3 xAxis_proj = {xAxis.x, xAxis.y, 0.0};

    // angle of projected x-axis to original x-axis
    double angle_z = (2 * M_PI) - acos(dot(xAxis_proj, vec3(1,0,0))
                                       / (length(xAxis_proj) * length(vec3(1,0,0))));
    if (isnan(angle_z)){
        angle_z = 0;
    }

    rollPlane(angle_z);

    /// 2: rotation around Y axis
    mat3 ry(1);
    ry(0, 0) = std::cos(angle);
    ry(0, 2) = std::sin(angle);
    ry(2, 0) = -std::sin(angle);
    ry(2, 2) = std::cos(angle);

    plane_ = ry * plane_;

    /// 3: roll back from XZ plane
    rollPlane(-angle_z);
}

/// roll ///
void Lsystem::rollPlane(double rollAngle){
//    rollAngle = rollAngle * M_PI/180;

    mat3 rz(1);
    rz(0, 0) = std::cos(rollAngle);
    rz(0, 1) = -std::sin(rollAngle);
    rz(1, 0) = std::sin(rollAngle);
    rz(1, 1) = std::cos(rollAngle);

    plane_ = rz * plane_;
}