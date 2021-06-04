//
// Created by hyde on 5/13/21.
//

#ifndef SYNTHESIS_PROJECT_ADTREE_L_BRANCH_H
#define SYNTHESIS_PROJECT_ADTREE_L_BRANCH_H

#endif //SYNTHESIS_PROJECT_ADTREE_L_BRANCH_H

#include <iostream>
#include <easy3d/core/graph.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/types.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/fileio/graph_io.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/algo/remove_duplication.h>
#include <easy3d/util/file_system.h>
#include <nlohmann/json.hpp>

#include <vector>
#include <map>
#include <string>
#include <sstream>
#include "AdTree/skeleton.h"
#include "Test_L_system/StringList.h"

class Lbranch {
public:
//    Lbranch();
    Lbranch(Skeleton* skl, float th_d, float th_x, float th_y);
//    ~Lbranch();

    struct BranchNode {
//        BranchNode();
//        ~BranchNode();
        // Attributes
        unsigned degree;
//        unsigned visit_time=0;
        int visit_time = 0;
        easy3d::vec3 cVert;
        size_t pre;
        std::vector<size_t> nexts;
    };

    bool notleaf(size_t vid);
    void print_detail();
    std::vector<size_t> find_next(size_t vid);
    void build_branches();
    void build_lstr();

    std::vector<std::string> return_Ls() {return Ls; }
    std::map<size_t, BranchNode> return_pool() {return pool; }

    //TODO: (Now form Noortje)

    std::vector<float> compute_nodes(size_t parent,size_t child) ;
    float getZAngle(easy3d::vec3 vec);
    float getYAngle(easy3d::vec3 vec);

    std::string make_nstr(std::vector<float> results) ;

    std::string make_lstr(std::vector<size_t> branch);

    //TODO:

//    std::vector<std::vector<size_t>> grow(std::vector<std::vector<size_t>> branches);


typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SGraphVertexProp, SGraphEdgeProp > Graph;

private:
    // Attributes
    std::vector<std::string> Ls;
    Skeleton* skl;
    Graph graph;
    size_t root;
    std::vector<size_t> vs;
    std::vector<BranchNode> nodes;
    std::map<size_t, BranchNode> pool;
    std::vector<std::vector<size_t>> branches;
    StringList strlist;
    float th_d;
    float th_x;
    float th_y;
};

Lbranch::Lbranch(Skeleton *skeleton, float th_d, float th_x, float th_y) {
    skl = skeleton;
    graph = skeleton->get_simplified_skeleton();
    root = skeleton->get_root();
    this->th_d = th_d; this->th_x=th_x; this->th_y = th_y;
    std::pair<SGraphVertexIterator, SGraphVertexIterator> vi = boost::vertices(graph);
    for (auto vit = vi.first; vit != vi.second; ++vit){
        if (boost::degree(*vit, graph)!=0) {
            vs.push_back(*vit);
            BranchNode temp;
            temp.degree = boost::degree(*vit, graph);
            temp.pre = graph[*vit].nParent;
            temp.cVert = graph[*vit].cVert;
            if(notleaf(*vit)) {
                temp.nexts = find_next(*vit);
            }
            nodes.push_back(temp);
        }
    }
    for (int i=0; i<vs.size(); ++i){
        pool.insert(std::make_pair(vs[i], nodes[i]));
    }
}

bool Lbranch::notleaf(size_t vid) {
    if (graph[vid].nParent != vid && boost::degree(vid, graph)==1) return false;
    return true;
}

std::vector<size_t> Lbranch::find_next(size_t vid) {
    std::pair<Graph::out_edge_iterator, Graph::out_edge_iterator> outei = boost::out_edges(vid, graph);
    std::vector<size_t> nexts_;
    for (auto eit = outei.first; eit!=outei.second; ++eit){
        if (boost::target(*eit, graph)!=graph[vid].nParent) nexts_.push_back(boost::target(*eit, graph));
    }
    return nexts_;
}

void Lbranch::print_detail() {
//    Graph& graph = *(const_cast<Graph*>(&skl->get_simplified_skeleton()));

//    for (auto const & mit : pool){
//        std::cout << "node: " << mit.first << " (degree: ";
//        std::cout << mit.second.degree << "): -->";
//        for (auto next:mit.second.nexts){
//            std::cout << next << " ";
//        }
//        std::cout <<std::endl;
//    }
//    for (auto B:Ls){
//        std::cout << B << std::endl;
//    }
    for (auto b:branches){
        for (auto n:b){
            std::cout << n << " ";
        }
        std::cout << std::endl;
//        std::cout << make_lstr(b) << std::endl;
    }
    strlist.printLstr();
}

void Lbranch::build_branches() {

    std::vector<size_t> wait_list;
    wait_list.push_back(root);
    while (!wait_list.empty()){
        size_t root_ = wait_list.back();
        std::string B;
        std::vector<size_t> branch;
        if (root_ == root) {
            B += "R --> ";
        }
        else B+= "N"+std::to_string(root_)+"_"+std::to_string(pool[root_].degree-1-pool[root_].visit_time)+" --> ";
        branch.push_back(root_);

        size_t next_ = pool[root_].nexts[pool[root_].visit_time];
        pool[root_].visit_time+=1;
        if (pool[root_].degree-1 <= pool[root_].visit_time) {
//            wait_list.erase(std::find(wait_list.begin(), wait_list.end(), root_));
            wait_list.pop_back();
        }
        while (notleaf(next_)){
            int ns = pool[next_].degree-1 - pool[next_].visit_time;
            if (ns>1){
                B+="(";
                for (int i=pool[next_].visit_time; i<ns; ++i){
                    B+= "N"+std::to_string(next_)+"_"+std::to_string(i+1)+" ";
                }
                B+=") --> ";
            }
            else B+= "N"+std::to_string(next_)+" --> ";
            branch.push_back(next_);
            if (pool[next_].degree-2 > pool[next_].visit_time) {
                wait_list.push_back(next_);
            }

            pool[next_].visit_time += 1;
            next_ = pool[next_].nexts[pool[next_].visit_time-1];
        }
        B += ("L" + std::to_string(next_));
        branch.push_back(next_);
        pool[next_].visit_time += 1;
        Ls.push_back(B);
        branches.push_back(branch);
    }
}



// TODO:
//std::string Lbranch::make_rule(){
//    std::string rule;
//    return rule;
//}

void Lbranch::build_lstr() {
    std::vector<size_t> wait_list;
    std::vector<StringList::LstrNode*> node_list;
    wait_list.push_back(root);
    node_list.push_back(strlist.get_head());
    while (!wait_list.empty()){
        size_t root_ = wait_list.back();
        StringList::LstrNode* head_ = node_list.back();
        StringList::LstrNode* p = head_;
        std::string L;
        std::vector<size_t> branch;
//        if (root_ == root) {
//            B += "R --> ";
//        }
//        else B+= "N"+std::to_string(root_)+"_"+std::to_string(pool[root_].degree-1-pool[root_].visit_time)+" --> ";
        branch.push_back(root_);
//        strlist.insert()

        size_t next_ = pool[root_].nexts[pool[root_].visit_time];
        pool[root_].visit_time+=1;
        if (pool[root_].degree-1 <= pool[root_].visit_time) {
//            wait_list.erase(std::find(wait_list.begin(), wait_list.end(), root_));
            wait_list.pop_back();
            node_list.pop_back();
        }
        while (notleaf(next_)){

            p = strlist.insert(p, make_nstr(compute_nodes(branch.back(), next_)));
            branch.push_back(next_);

            if (pool[next_].degree-2 > pool[next_].visit_time) {
                wait_list.push_back(next_);
                p = strlist.insert(p, "[");
                node_list.push_back(p);
                p = strlist.insert(p, "]");
            }

            pool[next_].visit_time += 1;
            next_ = pool[next_].nexts[pool[next_].visit_time-1];
        }
        // TODO: grow?
        strlist.insert(p, make_nstr(compute_nodes(branch.back(), next_)));
        branch.push_back(next_);
        pool[next_].visit_time += 1;
        branches.push_back(branch);
    }
}

std::vector<float> Lbranch::compute_nodes(size_t parent,size_t child) {

    // All from Noortje

    // TODO: rotation and roll? (my version)
    // tan
    std::vector<float> results= {0.0, 0.0, 0.0};
    easy3d::vec3 startV = pool[parent].cVert;
    easy3d::vec3 nextV = pool[child].cVert;
    easy3d::vec3 preV = pool[pool[parent].pre].cVert;

//    pool[branch.back()].cVert, pool[next_].cVert

//    easy3d::vec3 root_c = pool[root].cVert;
//    easy3d::vec3 ori = parent;
    float branch_length = easy3d::distance(startV, nextV);

    easy3d::vec3 to_target = nextV-startV;
    easy3d::vec3 to_origin = startV-preV;

    easy3d::vec3 xaxis = {1, 0, 0};
    easy3d::vec3 yaxis = {0, 1, 0};
    easy3d::vec3 zaxis = {0, 0, 1};

    easy3d::vec3 to_origin_xy = {to_origin.x, to_origin.y, 0};
    easy3d::vec3 to_target_xy = {to_target.x, to_target.y, 0};

    float angle_z_orig = getZAngle(to_origin_xy);
    float angle_z_target = getZAngle(to_target_xy);

    easy3d::vec3 to_origin_xz = easy3d::mat3::rotation(zaxis, -angle_z_orig) * to_origin;
    easy3d::vec3 to_target_xz = easy3d::mat3::rotation(zaxis, -angle_z_target) * to_target;

    float angle_y_orig = getYAngle(to_origin_xz);
    float angle_y_target = getYAngle(to_target_xz);

    float angle_diff_z = angle_z_target - angle_z_orig;
    float angle_diff_y = angle_y_target - angle_y_orig;

    if (abs(2 * M_PI - angle_diff_y) > 0.0001 && (abs(0 - angle_diff_y) > 0.0001)) {
        results[0] = angle_diff_y;
    }
    if (abs(2 * M_PI - angle_diff_z) > 0.0001 && (abs(0 - angle_diff_z) > 0.0001)) {
        results[1] = angle_diff_z;
    }
    results[2] = branch_length;

    return results;
}

float Lbranch::getZAngle(easy3d::vec3 vec){
    easy3d::vec3 xaxis = {1, 0, 0};
    float angle_z;

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


float Lbranch::getYAngle(easy3d::vec3 vec){
    easy3d::vec3 xaxis = {1, 0, 0};
    float angle_y;

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

std::string Lbranch::make_nstr (std::vector<float> results){
    float angle_y = results[0];
    float angle_z = results[1];
    float distance = results[2];
    int accuracy = 2;
    std::string nstr;

    bool degree_ = true;
    if (degree_){
        angle_y = results[0] / (M_PI / 180);
        angle_z = results[1] / (M_PI / 180);
    }

    // All from Noortje

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
//    if (std::abs(y_d)>th_y) {
//        int n = (int) (y_d / th_y);
//        if (n > 0) {
//            std::stringstream ss;
//            for (int i = 0; i < n; ++i) {
//                ss << '>';
//            }
//            nstr += ss.str();
//        } else {
//
//            std::stringstream ss;
//            for (int i = 0; i < n; ++i) {
//                ss << '<';
//            }
//            nstr += ss.str();
//        }
//    }
//
//    if (std::abs(x_d)>th_x){
//        if (x_d>0) nstr+="-";
//        else nstr += "+";
//    }
//    int nf = std::ceil(dis/th_d);
//    std::stringstream ss;
//    for (int i=0; i<nf; ++i){
//        ss << 'F';
//    }
//    nstr+=ss.str();
    return nstr;
//    return "-<F";
}

std::string Lbranch::make_lstr(std::vector<size_t> branch) {
    std::string lstr;
    int n = branch.size();
    // TODO: add markers
    for (int i=0; i<n-1; ++i){
        lstr += make_nstr(compute_nodes(branch[i], branch[i+1]));
    }
    return lstr;
}