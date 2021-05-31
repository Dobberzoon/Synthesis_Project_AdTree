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
        std::vector<size_t> nexts;
    };

    bool notleaf(size_t vid);
    void print_detail();
    std::vector<size_t> find_next(size_t vid);
    void build_branches();
    void build_lstr();

    std::vector<std::string> return_Ls() {return Ls; }
    std::map<size_t, BranchNode> return_pool() {return pool; }

    //TODO:

    std::vector<float> compute_nodes(easy3d::vec3 parent,easy3d::vec3 child) ;
    std::string make_nstr(std::vector<float> results) ;

    std::string make_lstr(std::vector<size_t> branch);

    //TODO:

//    std::vector<std::vector<size_t>> grow(std::vector<std::vector<size_t>> branches);



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
    strlist.traverse();
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

//            int i = 0;
//            while(pool[next_].degree-2 < pool[next_].visit_time) {
//                std::string lstr;
//                branch.push_back(next_);
//                lstr += make_nstr(compute_nodes(pool[next_]., pool[next_].cVert))
//            }

//            int ns = pool[next_].degree-1 - pool[next_].visit_time;
//            if (ns>1){
//                B+="(";
//                for (int i=pool[next_].visit_time; i<ns; ++i){
//                    B+= "N"+std::to_string(next_)+"_"+std::to_string(i+1)+" ";
//                }
//                B+=") --> ";
//            }
//            else B+= "N"+std::to_string(next_)+" --> ";
            p = strlist.insert(p, make_nstr(compute_nodes(pool[branch.back()].cVert, pool[next_].cVert)));
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
//        B += ("L" + std::to_string(next_));
        // TODO: grow?
        p = strlist.insert(p, make_nstr(compute_nodes(pool[branch.back()].cVert, pool[next_].cVert)));
        branch.push_back(next_);
        pool[next_].visit_time += 1;
//        Ls.push_back(B);
        branches.push_back(branch);
    }
}

std::vector<float> Lbranch::compute_nodes(easy3d::vec3 parent,easy3d::vec3 child) {

    // TODO: rotation and roll?
    // tan
    std::vector<float> results;
    float dis = easy3d::distance(parent, child);
    results.push_back(dis);
    float x_d = (child.x-parent.x)/dis;
    float y_d = (child.y-parent.y)/dis;
    results.push_back(x_d);
    results.push_back(y_d);
//    std::cout << results[0] << ", " << results[1] << ", " << results[2] << std::endl;
    return results;
}

std::string Lbranch::make_nstr (std::vector<float> results){
    float dis = results[0];
    float x_d = results[1];
    float y_d = results[2];
    std::string nstr;
    if (std::abs(y_d)>th_y) {
        int n = (int) (y_d / th_y);
        if (n > 0) {
            std::stringstream ss;
            for (int i = 0; i < n; ++i) {
                ss << '>';
            }
            nstr += ss.str();
        } else {

            std::stringstream ss;
            for (int i = 0; i < n; ++i) {
                ss << '<';
            }
            nstr += ss.str();
        }
    }

    if (std::abs(x_d)>th_x){
        if (x_d>0) nstr+="-";
        else nstr += "+";
    }
    int nf = std::ceil(dis/th_d);
    std::stringstream ss;
    for (int i=0; i<nf; ++i){
        ss << 'F';
    }
    nstr+=ss.str();
    return nstr;
//    return "F";
}

std::string Lbranch::make_lstr(std::vector<size_t> branch) {
    std::string lstr;
    int n = branch.size();
    // TODO: add markers
    for (int i=0; i<n-1; ++i){
        lstr += make_nstr(compute_nodes(pool[branch[i]].cVert, pool[branch[i+1]].cVert));
    }
    return lstr;
}