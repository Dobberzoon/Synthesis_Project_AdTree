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
#include <map>

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

    struct LBranch {
        std::string rotationSign;
        double rotationDegree;
        std::string rollSign;
        double rollDegree;
        double distance;
    };


    gLsystem();
    void readSkeleton(Skeleton* skeleton, bool deg);

    void traverse(SGraphVertexDescriptor prevV,
                  SGraphVertexDescriptor startV,
                  Skeleton *skeleton);

    std::tuple<double, double, double> moveToNext(SGraphVertexDescriptor startV,
                                                  SGraphVertexDescriptor nextV,
                                                  Skeleton *skeleton);

    void writeMovement(SGraphVertexDescriptor startV,
                       SGraphVertexDescriptor nextV,
                       Skeleton *skel,
                       int accuracy,
                       bool fast);

    double getZAngle(easy3d::vec3 vec);

    double getYAngle(easy3d::vec3 vec);

    void outputLsys(const std::string& out_type, const std::string& path);

    void printLsystem();

    void lsysToJson(const std::string &filenamel);

    void lsysToText(const std::string &filename);

    // TODO: about to grow

    bool sprout(int pos, SGraphVertexDescriptor vid, Skeleton *skel);
    bool fast(SGraphVertexDescriptor vid);

    void buildRules(Skeleton *skel, int accuracy);
    std::string selectRule(SGraphVertexDescriptor startV,
                           SGraphVertexDescriptor nextV,
                           Skeleton *skel);

    std::vector<size_t> findNext(size_t vid, Skeleton *skel);
    void buildBranches(Skeleton *skel);
    bool notLeaf(size_t vid, Skeleton *skel);

    void printSth(Skeleton *skel);

    //grow parameters
    int sprout_pos = 1;
    float grow_co = 0.1;
    float grow_sp = 0.1;
    float ratio = 3.0;

private:
    std::string Lstring_;
    std::string axiom_;
    bool degrees_ = true;
    int rec_ = 1;

    // default values
    float forward_ = 3;
    float rotation_ = 22.5;
    float roll_ = 20;

    //trunk values;
    easy3d::vec3 anchor_ = {0,0,0};
    float radius_ = 0.2;

    //Strs
//    StringList strlist;
//    std::vector<StringList::LstrNode*> strnodes;
//    StringList::LstrNode* head;


    //branches
    std::map<size_t, int> node_pos;
    std::vector<LBranch> last_branches;

    // grow rules
    std::vector<std::string> grow_rules;
};

using namespace boost;
using namespace easy3d;

gLsystem::gLsystem()
{
    Lstring_ = "";
    axiom_ = "";
    degrees_ = false;
//    head = strlist.get_head();
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
    j["rules"] = {{"A", grow_rules[0]},
                  {"B", grow_rules[1]},
                  {"C", grow_rules[2]},
                  {"D", grow_rules[3]}};        // empty for now
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
    radius_ = skel->getRadius()*(1+grow_sp*grow_co);
    anchor_ = skel->getAnchor();

    // l-branches
    buildBranches(skel);

    // rules
    buildRules(skel, 3);

    /// convert skeleton to Lsystem
    SGraphVertexDescriptor root = skel->get_root();
    vec3 coords_root = skel->get_simplified_skeleton()[root].cVert;
    traverse(root, root, skel);
//    Lstring_ = strlist.get_ls();
//    printSth(skel);

    std::cout << "converting to L-system: done" << std::endl;

    // todo: add more parameters to the L-system (branch diameters, subtrees, ...)
}


void gLsystem::traverse(SGraphVertexDescriptor prevV,
                       SGraphVertexDescriptor startV,
                       Skeleton *skel){
    // write movement from prevV to nextV to Lstring
    // will not write when prevV is a leaf or the root (preventing doubles and the root writing to itself)
    if (sprout(sprout_pos, prevV, skel)) {
        Lstring_ += selectRule(prevV, startV, skel);
//        Lstring_+="B";
    }
    writeMovement(prevV, startV, skel, 3, fast(prevV));

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
        if (out_degree(startV, skel->get_simplified_skeleton()) == 1 ||
        out_degree(startV, skel->get_simplified_skeleton()) == 2) {
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


void gLsystem::writeMovement(SGraphVertexDescriptor startV,
                            SGraphVertexDescriptor nextV,
                            Skeleton *skel,
                            int accuracy,
                            bool fast){
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
    if (fast) distance = distance*(1+grow_sp*ratio);
    else distance = distance*(1+grow_sp);

    // todo: generalisation

    /// write rotation
    // rounded to [accuracy] decimals
    if (angle_y > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << angle_y;
        std::string angle_y_string = ss.str();
        Lstring_ += "+(" + angle_y_string + ")";
    }
    if (angle_y < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << abs(angle_y);
        std::string angle_y_string = ss.str();
        Lstring_ += "-(" + angle_y_string + ")";
    }
    /// write roll
    if (angle_z > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << angle_z;
        std::string angle_z_string = ss.str();
        Lstring_ += ">(" + angle_z_string + ")";
    }
    if (angle_z < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << abs(angle_z);
        std::string angle_z_string = ss.str();
        Lstring_ += "<(" + angle_z_string + ")";
    }
    /// write forward
    if (distance > 0) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << distance;
        std::string dist_string = ss.str();
        Lstring_ += "F(" + dist_string + ")";
    }
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




bool gLsystem::sprout(int pos, SGraphVertexDescriptor vid, Skeleton *skel) {
    if (node_pos.count(vid)!=0 && boost::degree(vid, skel->get_simplified_skeleton())==2 &&
    skel->get_simplified_skeleton()[vid].nParent!=skel->get_root()){
//        switch (pos) {
//            case 1:
//                if (node_pos[vid]==1) return true;
////                break;
//            case 2:
//                if (node_pos[vid]==2) return true;
////                break;
//            case 3:
//                if (node_pos[vid]==3) return true;
////                break;
//            case 4:
//                if (node_pos[vid]==4) return true;
////                break;
//        }
        if (node_pos[vid]==pos) return true;
//        return false;
    }
    return false;
}

bool gLsystem::fast(SGraphVertexDescriptor vid) {
    if (node_pos.count(vid)!=0){
        if (node_pos[vid]<4) return true;
    }
    return false;
}


void gLsystem::buildBranches(Skeleton *skel) {
    std::vector<size_t> vs;
    std::vector<BranchNode> nodes;
    std::map<size_t, BranchNode> pool;
    std::vector<std::vector<size_t>> branches;
    std::pair<SGraphVertexIterator, SGraphVertexIterator> vi = boost::vertices(skel->get_simplified_skeleton());

//    int count_leaf =0;

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

//            std::cout << *vit << " is a leaf? " << !notLeaf(*vit, skel) << std::endl;
//            if (!notLeaf(*vit, skel)) count_leaf++;
        }
    }
//    std::cout << count_leaf << "leaves." << std::endl;

    for (int i=0; i<vs.size(); ++i){
        pool.insert(std::make_pair(vs[i], nodes[i]));
    }
    std::vector<size_t> wait_list;
    wait_list.push_back(skel->get_root());
    while (!wait_list.empty()){
        size_t root_ = wait_list.back();
        std::vector<size_t> branch;
        branch.push_back(root_);

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
//            if (pool[branch[i]].degree==2 && skel->get_simplified_skeleton()[branch[i]].nParent!=skel->get_root()){
//            std::cout << branch[i] << " ";
            if (node_pos.count(branch[i])!=0 && node_pos[branch[i]] < branch.size()-i-1) continue;
//                node_pos.insert(std::make_pair(branch[i], branch.size()-i-1));
//                if (node_pos[branch[i]] < branch.size()-i-1) continue;

            else node_pos[branch[i]] = branch.size()-i-1;

        }
//        std::cout << std::endl;
        branches.push_back(branch);
    }

//    std::cout << branches.size() << " branches" << std::endl;

//    for (auto b:branches){
//        for (auto n:b){
//            std::cout << n << "(" << boost::degree(n, skel->get_simplified_skeleton()) << ")" << " ";
//        }
//        std::cout << std::endl;
//    }

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

void gLsystem::buildRules(Skeleton *skel, int accuracy) {
    for (auto it=node_pos.begin(); it!=node_pos.end(); ++it){
        if (it->second==1){
            std::tuple<double, double, double> movement = moveToNext(it->first, findNext(it->first, skel)[0], skel);
            double angle_y = std::get<0>(movement);
            double angle_z = std::get<1>(movement);
            // option to output as degrees instead of radians
            if (degrees_){
                angle_y = std::get<0>(movement) / (M_PI / 180);
                angle_z = std::get<1>(movement) / (M_PI / 180);
            }
            double distance = std::get<2>(movement);

            LBranch lbranch;

            if (angle_y > 0){
                lbranch.rotationSign = "+";
                lbranch.rotationDegree = angle_y;
            }
            if (angle_y < 0){
                lbranch.rotationSign = "-";
                lbranch.rotationDegree = angle_y;
            }
            // roll
            if (angle_z > 0){
                lbranch.rollSign = ">";
                lbranch.rollDegree = angle_z;
            }
            if (angle_z < 0){
                lbranch.rollSign = "<";
                lbranch.rollDegree = angle_z;
            }
            // forward
            if (distance > 0) {
                lbranch.distance = distance;
            }
            last_branches.push_back(lbranch);
//            std::cout << distance << " " << std::endl;
        }
    }
    int count_rotation = 0;
    int count_roll = 0;
    double sum_rotation = 0.0;
    double avg_rotation = 0.0;
    double sum_roll = 0.0;
    double avg_roll = 0.0;
    int count_rotation_ = 0;
    int count_roll_ = 0;
    double sum_rotation_ = 0.0;
    double avg_rotation_ = 0.0;
    double sum_roll_ = 0.0;
    double avg_roll_ = 0.0;
    int count_dis = 0;
    double sum_dis = 0.0;
    double avg_dis = 0.0;

    for (auto lb:last_branches){
        if (lb.rotationSign=="+"){
            count_rotation++;
            sum_rotation += lb.rotationDegree;
        }
        else if (lb.rotationSign=="-"){
            count_rotation_++;
            sum_rotation_ += lb.rotationDegree;
        }
        if (lb.rollSign==">"){
            count_roll++;
            sum_roll += lb.rollDegree;
        }
        else if (lb.rollSign=="<"){
            count_roll_++;
            sum_roll_ += lb.rollDegree;
        }
        count_dis++;
        sum_dis += lb.distance;
    }


    if (count_rotation!=0) avg_rotation = sum_rotation/count_rotation;
    if (count_rotation_!=0) avg_rotation_ = sum_rotation_/count_rotation_;
    if (count_roll!=0) avg_roll = sum_roll/count_roll;
    if (count_roll_!=0) avg_roll_ = sum_roll_/count_roll_;
    avg_dis = sum_dis/count_dis;

    double grow_rotation, grow_rotation_, grow_roll, grow_roll_;
///*  A test:
    if (degrees_){
        if (avg_rotation+30>360) grow_rotation = avg_rotation-30;
        else grow_rotation = avg_rotation+30;
        if (avg_rotation_-30 < -360) grow_rotation_ = avg_rotation_+30;
        else grow_rotation_ = avg_rotation_-30;
        if (avg_roll+30>360) grow_roll = avg_roll-30;
        else grow_roll = avg_roll+30;
        if (avg_roll_-30< -360) grow_roll_ = avg_roll_+30;
        else grow_roll_ = avg_roll_-30;
    }

    else {
        if (avg_rotation+30*M_PI/180>2*M_PI) grow_rotation = avg_rotation-30*M_PI/180;
        else grow_rotation = avg_rotation+30*M_PI/180;
        if (avg_rotation_-30*M_PI/180< -2*M_PI) grow_rotation_ = avg_rotation_+30*M_PI/180;
        else grow_rotation_ = avg_rotation_+30*M_PI/180;
        if (avg_roll+30*M_PI/180>2*M_PI) grow_roll = avg_roll-30*M_PI/180;
        else grow_roll = avg_roll+30*M_PI/180;
        if (avg_roll_-30*M_PI/180< -2*M_PI) grow_roll_ = avg_roll_+30*M_PI/180;
        else grow_roll_ = avg_roll_-30*M_PI/180;
    }
//*/

//    std::cout << avg_rotation << " " << avg_rotation_ << std::endl;
//    grow_rotation = avg_rotation;
//    grow_rotation_ = avg_rotation_;
//    grow_roll = avg_roll;
//    grow_roll_ = avg_roll_;
//    int accuracy = 3;
    std::string r1, r2, r3, r4;
    std::stringstream s1, s2, s3, s4;

    s1 << "[+(";
    s1 << std::fixed << std::setprecision(accuracy) << grow_rotation;
    s1 << ")>(";
    s1 << std::fixed << std::setprecision(accuracy) << grow_roll;
    s1 << ")F(";
    s1 << std::fixed << std::setprecision(accuracy) << avg_dis;
    s1 << ")]";
    r1 = s1.str();
//    std::cout << r1 << std::endl;

    s2 << "[-(";
    s2 << std::fixed << std::setprecision(accuracy) << std::abs(grow_rotation_);
    s2 << ")>(";
    s2 << std::fixed << std::setprecision(accuracy) << grow_roll;
    s2 << ")F(";
    s2 << std::fixed << std::setprecision(accuracy) << avg_dis;
    s2 << ")]";
    r2 = s2.str();
//    std::cout << r2 << std::endl;


    s3 << "[+(";
    s3 << std::fixed << std::setprecision(accuracy) << grow_rotation;
    s3 << ")<(";
    s3 << std::fixed << std::setprecision(accuracy) << std::abs(grow_roll_);
    s3 << ")F(";
    s3 << std::fixed << std::setprecision(accuracy) << avg_dis;
    s3 << ")]";
    r3 = s3.str();
//    std::cout << r3 << std::endl;


    s4 << "[-(";
    s4 << std::fixed << std::setprecision(accuracy) << std::abs(grow_rotation_);
    s4 << ")<(";
    s4 << std::fixed << std::setprecision(accuracy) << std::abs(grow_roll_);
    s4 << ")F(";
    s4 << std::fixed << std::setprecision(accuracy) << avg_dis;
    s4 << ")]";
    r4 = s4.str();
//    std::cout << r4 << std::endl;


    grow_rules.push_back(r1);
    grow_rules.push_back(r2);
    grow_rules.push_back(r3);
    grow_rules.push_back(r4);

}

std::string gLsystem::selectRule(SGraphVertexDescriptor startV, SGraphVertexDescriptor nextV, Skeleton *skel) {
//    return "";
    std::tuple<double, double, double> movement = moveToNext(startV, nextV, skel);
    if (std::get<0>(movement)>0 && std::get<1>(movement)>0) return "A";
    else if (std::get<0>(movement)<0 && std::get<1>(movement)>0) return "B";
    else if (std::get<0>(movement)>0 && std::get<1>(movement)<0) return "C";
    else if (std::get<0>(movement)<0 && std::get<1>(movement)<0) return "D";
    return "";
}


void gLsystem::printSth(Skeleton *skel) {
//    int count_sprouts = 0;
//    for (auto it=node_pos.begin(); it!=node_pos.end(); ++it){
//        if (sprout(sprout_pos, it->first, skel)){
//            std::cout << it->first << ": " << it->second << " ";
//            count_sprouts++;
//        }

//        std::cout << it->first << ": " << it->second << " ";
//        std::cout << " sprout? " << sprout(sprout_pos, it->first, skel) << std::endl;
//    }
//    std::cout << count_sprouts << " sprouts." << std::endl;
//    for (auto it:strnodes){
//        std::cout << it->Lstr << std::endl;
//    }
//    strlist.printLstr();
//    std::cout << Lstring_ << std::endl;
}

#endif //L_SYSTEM_GLSYSTEM_H
