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


void Lsystem::moveToNext(SGraphVertexDescriptor startV, SGraphVertexDescriptor nextV, Skeleton *skel) {
    // get previous node (parent of start)
    // todo: this may break if start is the origin?
    SGraphVertexDescriptor prevV = skel->get_simplified_skeleton()[startV].nParent;

    // get coordinates of the two nodes
    vec3 coords_start = skel->get_simplified_skeleton()[startV].cVert;
    vec3 coords_next = skel->get_simplified_skeleton()[nextV].cVert;
    vec3 coords_prev = skel->get_simplified_skeleton()[prevV].cVert;

    // get distance between the two nodes
    float branch_length = easy3d::distance(coords_start, coords_next);

    if (startV == 5 && nextV == 6) {
        std::cout << "\n---------- computing translation ----------" << std::endl;
        std::cout << "start node: (" << startV << ") " << coords_start
                  << " --> next: (" << nextV << ") " << coords_next
                  << " | previous: (" << prevV << ") " << coords_prev << std::endl;
    }

    /* angle/z-axis (before) origin
     * angle/z-axis (to) target
     *
     * set plane to rotation before origin
     *
     * project to zx plane
     * rotate around y-axis
     *
     * project to xy plane
     * roll around NEW z-axis
     *
     * NEW NEW z-axis should be tangent
     *
     * forward: distance * NEW NEW z-axis
    d*/

    /// compute vectors in coordinate system of startV
    // compute the tangent between the two nodes
    vec3 to_target = (coords_next - coords_start);   //.normalize()
//    easy3d::Vec<3, double> tt_easy3d = {tt.x, tt.y, tt.z};
//    easy3d::Vec<3, double> to_target= plane_ * tt_easy3d;
    // compute the previous z-axis
    vec3 to_origin = (coords_start - coords_prev);

//    std::cout << "current plane:\n" << plane_ << std::endl;
    if (startV == 5 && nextV == 6) {
        std::cout << "to_target: " << to_target << " , length: " << length(to_target) << std::endl;
        std::cout << "to_origin: " << to_origin << " , length: " << length(to_origin) << std::endl;
    }

    /*easy3d::Vec<3, double> xaxis = plane_.row(0);
    easy3d::Vec<3, double> yaxis = plane_.row(1);
    easy3d::Vec<3, double> zaxis = plane_.row(2);

    vec3 zaxis_orig = {0, 0, 1};

    double angle_coordsys = acos(dot(zaxis_orig, to_origin) / (length(zaxis_orig) * length(to_origin)));

    mat3 R;
    R = R.rotation(cross(zaxis_orig, to_origin), (2*M_PI - angle_coordsys));

    R = R.rotation(zaxis_orig, ((3/2)*M_PI));
    vec3 xaxis = R * to_origin;

    std::cout << "x axis: " << xaxis << std::endl;

    std::cout << "R[0, 0, 1] = " << R * zaxis_orig << std::endl;
    std::cout << "rotation matrix:\n" << R << std::endl;

    std::cout << "angle: " << angle_coordsys / (M_PI/180) << std::endl;


    // project tangent to 2D planes
    easy3d::Vec<3, double> tan_proj_zx = {to_target.x, zaxis.y, to_target.z};     // rotate
    easy3d::Vec<3, double> tan_proj_xy = {to_target.x, to_target.y, zaxis.z};     // roll

    /// rotate
//    double angle = acos((dot(coords_start, coords_next)) / (length(coords_start) * length(coords_next)));
    easy3d::Vec<3, double> proj_zx = {to_target.x, zaxis.y, to_target.z};

    / roll

    / set plane

    std::cout << "current plane:\n" << plane_ << std::endl;

    double angle_rad = asin(norm(cross(zaxis, tangent)));
//    double angle_rad = angle_deg * M_PI/180;
    double angle_deg = angle_rad / (M_PI/180);
    mat3 rot;
    rot = rot.rotation(cross(zaxis, tangent), angle_rad);

    std::cout << "length tangent: " << round(length(tangent)) << std::endl;
    std::cout << "tangent:         " << tangent << std::endl;
    std::cout << "rotation result: " << rot * zaxis << std::endl;
    std::cout << "ange rad: " << angle_rad << " , angle degrees: " << round(angle_deg) << std::endl;
    std::cout << "rotation matrix:\n" << rot << std::endl;*/

    /*vec3 xaxis = {1, 0, 0};
    vec3 yaxis = {0, 1, 0};
    vec3 zaxis = {0, 0, 1};

    /// rotate (y-axis)
    // project to xz plane
    easy3d::Vec<3, double> to_target_y = {to_target.x, 0, to_target.z};
    easy3d::Vec<3, double> to_origin_y = {to_origin.x, 0, to_origin.z};

    double angle_y = acos(dot(to_origin_y, to_target_y) / (length(to_origin_y) * length(to_target_y)));
    angle_y = (2*M_PI) - angle_y;   // counter clockwise rotation

    mat3 Ry(1);

    vec3 xrot = xaxis*cos(angle_y) - zaxis*sin(angle_y);
    vec3 zrot = xaxis*sin(angle_y) + zaxis*cos(angle_y);

    xrot.normalize();
    zrot.normalize();

    Ry.set_row(0, xrot);
    Ry.set_row(2, zrot);

    /// roll (z-axis)
    // incorporate previous rotation
    vec3 to_origin_roty = Ry * to_origin;

    // project to xy plane
    vec3 to_target_z = {to_target.x, to_target.y, 0};
    vec3 to_origin_z = {to_origin_roty.x, to_origin_roty.y, 0};

    double angle_z = acos(dot(to_origin_z, to_target_z) / (length(to_origin_z) * length(to_target_z)));
    angle_z = (2*M_PI) - angle_z;

    mat3 Rz(1);

    xrot = xrot*cos(angle_z) - yaxis*sin(angle_z);
    vec3 yrot = xrot*sin(angle_z) + yaxis*cos(angle_z);

    xrot.normalize();
    yrot.normalize();

    Rz.set_row(0, xrot);
    Rz.set_row(1, yrot);

    mat3 R;
//    R = R.rotation(0, angle_y, angle_z, 123);
//    mat3 Rz(1);
//    mat3 Ry(1);
    R = Rz * Ry;

    std::cout << "R * tan orig: " << R * to_origin << std::endl;
    std::cout << "Rz * tan orig: " << Rz * to_origin << std::endl;
    std::cout << "rotation matrix:\n" << R << std::endl;
}*/

    /*double yaw_orig = atan2(to_origin.z, to_origin.x);
    double pitch_orig = atan2(sqrt(pow(to_origin.z, 2) + pow(to_origin.x, 2)), to_origin.y) + M_PI;

    double yaw_target = atan2(to_target.z, to_target.x);
    double pitch_target = atan2(sqrt(pow(to_target.z, 2) + pow(to_target.x, 2)), to_target.y) + M_PI;

    double yaw = yaw_target - yaw_orig;
    double pitch = pitch_target - pitch_orig;

    std::cout << "yaw: " << yaw << " , pitch: " << pitch << std::endl;*/

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
    // todo: reverse for actual rotation of Z-axis
    double angle_z_orig = acos(dot(to_origin_xy, xaxis) / (length(to_origin_xy) * length(xaxis)));
    double angle_z_target = acos(dot(to_target_xy, xaxis) / (length(to_target_xy) * length(xaxis)));

    /// get rotation around Y
    // rotate vectors to XZ plane
    vec3 to_origin_xz = easy3d::mat3::rotation(zaxis, -angle_z_orig) * to_origin;
    vec3 to_target_xz = easy3d::mat3::rotation(zaxis, -angle_z_target) * to_target;

    // find angle between planar vectors & x-axis
    double angle_y_orig = acos(dot(to_origin_xz, xaxis) / (length(to_origin_xz) * length(xaxis)));
    double angle_y_target = acos(dot(to_target_xz, xaxis) / (length(to_target_xz) * length(xaxis)));

    /// combine rotations
    mat3 R;
    mat3 Rz_orig;
    mat3 Ry_orig;

    if (startV == 5 && nextV == 6) {
        std::cout << "to_origin XY (z): " << to_origin_xy << std::endl;
        std::cout << "origin angle z: " << angle_z_orig / (M_PI/180) << std::endl;
        std::cout << "to_origin XZ (y): " << to_origin_xz << std::endl;
        std::cout << "origin angle y: " << angle_y_orig / (M_PI/180) << std::endl;

        std::cout << "\nto_target XY (z): " << to_target_xy << std::endl;
        std::cout << "target angle z: " << angle_z_target / (M_PI/180) << std::endl;
        std::cout << "to_target XZ (y): " << to_target_xz << std::endl;
        std::cout << "target angle y: " << angle_y_target / (M_PI/180) << std::endl;

        std::cout << "diff. angle z: " << (angle_z_target - angle_z_orig) / (M_PI/180) << std::endl;
        std::cout << "diff. angle y: " << (angle_y_target - angle_y_orig) / (M_PI/180) << std::endl;

        std::cout << "\nnormalized to_origin:  " << to_origin.normalize()
                  << "\nzaxis check to_origin: "
                  << easy3d::mat3::rotation(0, ((-1/2)*M_PI + angle_y_orig), (angle_z_orig), 123) * zaxis
                  << std::endl;
        std::cout << "normalized to_target:  " << to_target.normalize()
                  << "\nzaxis check to_target: "
                  << (easy3d::mat3::rotation(0, ((-1/2)*M_PI + angle_y_target), (angle_z_target), 123) * zaxis)
                  << std::endl;
    }

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