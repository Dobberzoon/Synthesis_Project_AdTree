//
// Created by hyde on 6/3/21.
//

#ifndef L_SYSTEM_GLSYSTEM_H
#define L_SYSTEM_GLSYSTEM_H

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

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <vector>

#include "AdTree/skeleton.h"
#include "3rd_party/nlohmann/json.hpp"
#include "StringList.h"


class gLsystem {
public:
    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SGraphVertexProp, SGraphEdgeProp > Graph;
    struct BranchNode {
        // Attributes
        unsigned degree;
        int visit_time = 0;
        easy3d::vec3 cVert;
        size_t pre;
        std::vector<size_t> nexts;
    };


    gLsystem();
    void readSkeleton(Skeleton* skeleton, bool deg);

    void traverse(SGraphVertexDescriptor prevV,
                  SGraphVertexDescriptor startV,
                  Skeleton *skeleton,
                  StringList::LstrNode* p);

    std::tuple<double, double, double> moveToNext(SGraphVertexDescriptor startV,
                                                  SGraphVertexDescriptor nextV,
                                                  Skeleton *skeleton);

    StringList::LstrNode* writeMovement(SGraphVertexDescriptor startV,
                       SGraphVertexDescriptor nextV,
                       Skeleton *skel,
                       int accuracy,
                       StringList::LstrNode* p);

    double getZAngle(easy3d::vec3 vec);

    double getYAngle(easy3d::vec3 vec);

    void outputLsys(const std::string& out_type, const std::string& path);

    void printLsystem();

    void lsysToJson(const std::string &filenamel);

    void lsysToText(const std::string &filename);

    // TODO: about to grow

    bool sprout(int pos, SGraphAdjacencyIterator vid, Skeleton *skel);


    std::vector<size_t> findNext(size_t vid, Skeleton *skel);
    void buildBranches(Skeleton *skel);
    bool notLeaf(size_t vid, Skeleton *skel);

private:
    std::string Lstring_;
    std::string axiom_;
    bool degrees_ = true;
    int rec_ = 0;

    // default values
    float forward_ = 3;
    float rotation_ = 22.5;
    float roll_ = 20;

    //trunk values;
    easy3d::vec3 anchor_ = {0,0,0};
    float radius_ = 0.2;

    //Strs
    StringList strlist;
    std::vector<StringList::LstrNode*> strnodes;
    StringList::LstrNode* head;

    //grow parameters
    int sprout_pos = 1;
    float grow_co = 0.01;
    float ratio = 2.0;

    //branches
    std::map<size_t, int> nodePos;
};

using namespace boost;
using namespace easy3d;

gLsystem::gLsystem()
{
    Lstring_ = "";
    axiom_ = "";
    degrees_ = false;
    head = strlist.get_head();
}

// todo: add rules
// todo: add radii


void gLsystem::printLsystem() {
    std::cout << "printing L-system..." << std::endl;
    std::cout << "string: " << Lstring_ << std::endl;
    std::cout << "axiom:  " << axiom_ << std::endl;
    std::cout << "printing L-system: done" << std::endl;
}


void gLsystem::lsysToJson(const std::string &filename) {
    std::cout << "L-system: writing to file..." << std::endl;

    nlohmann::json j;

    j["recursions"] = rec_;
    j["axiom"] = Lstring_;  // todo: this will at some point be axiom_
    j["rules"] = {};        // empty for now
    j["trunk"] = {{"anchor", {anchor_.x, anchor_.y, anchor_.z}},
                  {"radius", radius_}};

    j["dimensions"] = { {"forward", forward_},
                        {"rotation", rotation_},
                        {"roll", roll_} };

    j["degrees"] = degrees_;

//    std::cout << std::setw(4) << j << std::endl;
//    std::cout << "dir: " << output_dir + filename << std::endl;

    std::ofstream storageFile(filename);
    storageFile << std::setw(4) << j << std::endl;
    storageFile.close();

    std::cout << "writing to file: done" << std::endl;
}


void gLsystem::lsysToText(const std::string &filename){};


void gLsystem::readSkeleton(Skeleton *skel, bool deg) {
    std::cout << "\n---------- initializing L-system ----------" << std::endl;
    std::cout << "nr. vertices of simplified skeleton: " << num_vertices(skel->get_simplified_skeleton()) << std::endl;

    /// set parameters
    degrees_ = deg;
    radius_ = skel->getRadius();
    anchor_ = skel->getAnchor();

    // l-branches
    buildBranches(skel);

    /// convert skeleton to Lsystem
    SGraphVertexDescriptor root = skel->get_root();
    vec3 coords_root = skel->get_simplified_skeleton()[root].cVert;
    traverse(root, root, skel, head);

    std::cout << "converting to L-system: done" << std::endl;

    // todo: add more parameters to the L-system (branch diameters, subtrees, ...)
}


void gLsystem::traverse(SGraphVertexDescriptor prevV,
                        SGraphVertexDescriptor startV,
                        Skeleton *skel,
                        StringList::LstrNode* p){
    // write movement from prevV to nextV to Lstring
    // will not write when prevV is a leaf or the root (preventing doubles and the root writing to itself)
    StringList::LstrNode* p_ = writeMovement(prevV, startV, skel, 3, p);


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

        // depth-first shortest path search, copied from AdTree's method for making simplified_skeleton_
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
        // todo: only the first child is fastest, other children are just in rotation order (not sure if it matters)

        /// start node has one child: straight segment
        if (out_degree(startV, skel->get_simplified_skeleton()) == 1) {
            traverse(startV, nextV, skel, p_);
        }
            /// start node has multiple children: beginning of 2 or more branches
        else {
            // write the fastest child
            Lstring_ += "[";
            traverse(startV, nextV, skel, p_);
            Lstring_ += "]";

            // also write all the other children
            for (int nChild = 0; nChild < slower_children.size(); ++nChild) {
                Lstring_ += "[";
                traverse(startV, slower_children[nChild], skel, p_);
                Lstring_ += "]";
            }
        }
    }
}


std::tuple<double, double, double> gLsystem::moveToNext(SGraphVertexDescriptor startV,
                                                        SGraphVertexDescriptor nextV,
                                                        Skeleton *skel) {
    // end result: {angle around y-axis, angle anround z-axis, distance}


    std::tuple<double, double, double> movement{0, 0, 0};

    // get previous node (parent of start)
    SGraphVertexDescriptor prevV = skel->get_simplified_skeleton()[startV].nParent;

    /// get relative movement from start to next node
    // exclude parent as second node (next)
    // because is it's own parent, so vector between start & next would be 0
    if (nextV != skel->get_simplified_skeleton()[nextV].nParent) {
        // get coordinates of the two nodes
        vec3 coords_start = skel->get_simplified_skeleton()[startV].cVert;
        vec3 coords_next = skel->get_simplified_skeleton()[nextV].cVert;
        vec3 coords_prev = skel->get_simplified_skeleton()[prevV].cVert;

        // get distance between the two nodes
        float branch_length = easy3d::distance(coords_start, coords_next);

        /// compute vectors previous <--> start and start <--> next
        // compute the vector (difference) between the two nodes (start & next)
        vec3 to_target = (coords_next - coords_start);
        // compute the vector of the previous edge
        vec3 to_origin = (coords_start - coords_prev);

        vec3 xaxis = {1, 0, 0};
        vec3 yaxis = {0, 1, 0};
        vec3 zaxis = {0, 0, 1};

        /// get rotation around Z
        // project to xy plane
        vec3 to_origin_xy = {to_origin.x, to_origin.y, 0};
        vec3 to_target_xy = {to_target.x, to_target.y, 0};

        // find angle between planar vectors & x-axis, around the z-axis (radians)
        double angle_z_orig = getZAngle(to_origin_xy);
        double angle_z_target = getZAngle(to_target_xy);

        // todo: make 360 --> 0?
        // todo: 360+ or -360-?

        /// get rotation around Y
        // rotate vectors to XZ plane
        vec3 to_origin_xz = easy3d::mat3::rotation(zaxis, -angle_z_orig) * to_origin;
        vec3 to_target_xz = easy3d::mat3::rotation(zaxis, -angle_z_target) * to_target;

        // find angle between planar vectors & x-axis, around the y-axis
        double angle_y_orig = getYAngle(to_origin_xz);
        double angle_y_target = getYAngle(to_target_xz);

        /// get relative rotations
        double angle_diff_z = angle_z_target - angle_z_orig;
        double angle_diff_y = angle_y_target - angle_y_orig;

        // may still need these later, switched off for now
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


StringList::LstrNode* gLsystem::writeMovement(SGraphVertexDescriptor startV,
                             SGraphVertexDescriptor nextV,
                             Skeleton *skel,
                             int accuracy,
                             StringList::LstrNode* p){
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

    // todo: generalisation

    /// write rotation
    // rounded to [accuracy] decimals

    std::string nstr;

    if (angle_y > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << angle_y;
        std::string angle_y_string = ss.str();
        nstr += "+(" + angle_y_string + ")";
    }
    if (angle_y < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << abs(angle_y);
        std::string angle_y_string = ss.str();
        nstr += "-(" + angle_y_string + ")";
    }
    /// write roll
    if (angle_z > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << angle_z;
        std::string angle_z_string = ss.str();
        nstr += ">(" + angle_z_string + ")";
    }
    if (angle_z < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << abs(angle_z);
        std::string angle_z_string = ss.str();
        nstr += "<(" + angle_z_string + ")";
    }
    /// write forward
    if (distance > 0) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << distance;
        std::string dist_string = ss.str();
        nstr += "F(" + dist_string + ")";
    }

    p = strlist.insert(p, nstr);

}


double gLsystem::getZAngle(vec3 vec){
    vec3 xaxis = {1, 0, 0};
    double angle_z;

    // angle is dependent on what side of the x-axis (XY plane) the vector is
    if (vec.y < 0){
        angle_z = - acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }
    else{
        angle_z = acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }

    // zero angle returns nan, should be 0
    if (isnan(angle_z)) {
        angle_z = 0;
    }

    return angle_z;
}


double gLsystem::getYAngle(vec3 vec){
    vec3 xaxis = {1, 0, 0};
    double angle_y;

    // angle is dependent on what side of the x-axis (XZ plane) the vector is
    if (vec.z < 0) {
        angle_y = acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }
    else{
        angle_y = (2 * M_PI) - acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }

    // zero angle returns nan, should be 0
    if (isnan(angle_y)) {
        angle_y = 0;
    }

    return angle_y;
}


void gLsystem::outputLsys(const std::string& out_type, const std::string& path){
    if(out_type == "json"){
        lsysToJson(path);
    } else if (out_type == "txt"){
        lsysToText(path);
    }
}




bool gLsystem::sprout(int pos, SGraphAdjacencyIterator v, Skeleton *skel) {
    switch (pos) {
//        case 1:
    }
}

void gLsystem::buildBranches(Skeleton *skel) {
    std::vector<size_t> vs;
    std::vector<BranchNode> nodes;
    std::map<size_t, BranchNode> pool;
    std::vector<std::vector<size_t>> branches;
    std::pair<SGraphVertexIterator, SGraphVertexIterator> vi = boost::vertices(skel->get_simplified_skeleton());

    for (auto vit = vi.first; vit != vi.second; ++vit){
        if (boost::degree(*vit, skel->get_simplified_skeleton())!=0) {
            vs.push_back(*vit);
            BranchNode temp;
            temp.degree = boost::degree(*vit, skel->get_simplified_skeleton());
            temp.pre = skel->get_simplified_skeleton()[*vit].nParent;
            temp.cVert = skel->get_simplified_skeleton()[*vit].cVert;
            if(notLeaf(*vit, skel)) {
                temp.nexts = findNext(*vit, skel);
            }
            nodes.push_back(temp);
        }
    }
    for (int i=0; i<vs.size(); ++i){
        pool.insert(std::make_pair(vs[i], nodes[i]));
    }

    std::vector<size_t> wait_list;
    wait_list.push_back(skel->get_root());
    while (!wait_list.empty()){
        size_t root_ = wait_list.back();
        std::vector<size_t> branch;
//        branch.push_back(root_);

        size_t next_ = pool[root_].nexts[pool[root_].visit_time];
        pool[root_].visit_time+=1;
        if (pool[root_].degree-1 <= pool[root_].visit_time) {
//            wait_list.erase(std::find(wait_list.begin(), wait_list.end(), root_));
            wait_list.pop_back();
        }
        while (notLeaf(next_, skel)){
            branch.push_back(next_);

            if (pool[next_].degree-2 > pool[next_].visit_time) {
                wait_list.push_back(next_);
            }

            pool[next_].visit_time += 1;
            next_ = pool[next_].nexts[pool[next_].visit_time-1];
        }
        // TODO: grow?
        branch.push_back(next_);
        pool[next_].visit_time += 1;

        for (int i=0; i<branch.size(); i++){
            if (!(pool[branch[i]].degree>2 || branch[i]==skel->get_root())){
                nodePos.insert(std::make_pair(branch[i], branch.size()-i-1));
            }
        }
        branches.push_back(branch);
    }
}

bool gLsystem::notLeaf(size_t vid, Skeleton *skel) {
    if (skel->get_simplified_skeleton()[vid].nParent != vid && boost::degree(vid, skel->get_simplified_skeleton())==1) return false;
    return true;
}

std::vector<size_t> gLsystem::findNext(size_t vid, Skeleton *skel) {

    std::pair<Graph::out_edge_iterator, Graph::out_edge_iterator> outei = boost::out_edges(vid, skel->get_simplified_skeleton());
    std::vector<size_t> nexts_;
    for (auto eit = outei.first; eit!=outei.second; ++eit){
        if (boost::target(*eit, skel->get_simplified_skeleton())!=skel->get_simplified_skeleton()[vid].nParent) nexts_.push_back(boost::target(*eit, skel->get_simplified_skeleton()));
    }
    return nexts_;
}

#endif //L_SYSTEM_GLSYSTEM_H
