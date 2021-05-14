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

#include <vector>
#include <map>
#include <string>
#include "AdTree/skeleton.h"

class Lbranch {
public:
//    Lbranch();
    Lbranch(Skeleton* skl);
//    ~Lbranch();

    struct BranchNode {
//        BranchNode();
//        ~BranchNode();
        // Attributes
        unsigned degree;
//        unsigned visit_time=0;
        int visit_time = 0;
        std::vector<size_t> nexts;
    };

    bool notleaf(size_t vid);
    void print_detail();
    std::vector<size_t> find_next(size_t vid);
    void build_branches();

    std::vector<std::string> return_Ls() {return Ls; }
    std::map<size_t, BranchNode> return_pool() {return pool; }

    //TODO:
    std::string make_rule();

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
};

Lbranch::Lbranch(Skeleton *skeleton) {
    skl = skeleton;
    graph = skeleton->get_simplified_skeleton();
    root = skeleton->get_root();
    std::pair<SGraphVertexIterator, SGraphVertexIterator> vi = boost::vertices(graph);
//    std::cout << *vi.first << std::endl;
//    std::cout << "here" << std::endl;
    for (auto vit = vi.first; vit != vi.second; ++vit){
        if (boost::degree(*vit, graph)!=0) {
            vs.push_back(*vit);
            BranchNode temp;
            temp.degree = boost::degree(*vit, graph);
            if(notleaf(*vit)) {
                temp.nexts = find_next(*vit);
            }
            nodes.push_back(temp);
//            std::cout << *vit << std::endl;
//            nodes.push_back(temp);
//            pool.insert(std::make_pair(*vit, temp));
        }
    }
    for (int i=0; i<vs.size(); ++i){
        pool.insert(std::make_pair(vs[i], nodes[i]));
    }
}

bool Lbranch::notleaf(size_t vid) {
//    Graph& graph = *(const_cast<Graph*>(&skl->get_simplified_skeleton()));
//    Graph graph = skl->get_simplified_skeleton();
    if (graph[vid].nParent != vid && boost::degree(vid, graph)==1) return false;
    return true;
}

std::vector<size_t> Lbranch::find_next(size_t vid) {
//    Graph& graph = *(const_cast<Graph*>(&skl->get_simplified_skeleton()));
    std::pair<Graph::out_edge_iterator, Graph::out_edge_iterator> outei = boost::out_edges(vid, graph);
    std::vector<size_t> nexts_;
    for (auto eit = outei.first; eit!=outei.second; ++eit){
//        my_file << "-->" << boost::target(*eit, graph) << std::endl;
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
    for (auto B:Ls){
        std::cout << B << std::endl;
    }
//    for (auto b:branches){
//        for (auto n:b){
//            std::cout << n << " ";
//        }
//        std::cout << std::endl;
//    }
}

void Lbranch::build_branches() {

    std::vector<size_t> wait_list;
    wait_list.push_back(root);
    while (!wait_list.empty()){
//        std::cout << "here" << std::endl;
        size_t root_ = wait_list[0];
        std::string B;
        std::vector<size_t> branch;
        if (root_ == root) {
            B += "R --> ";
        }
        else B+= "N"+std::to_string(root_)+"_"+std::to_string(pool[root_].degree-1-pool[root_].visit_time)+" --> ";
        branch.push_back(root_);

//        std::cout << B << std::endl;

        size_t next_ = pool[root_].nexts[pool[root_].visit_time];
        pool[root_].visit_time+=1;
        if (pool[root_].degree-1 <= pool[root_].visit_time) {
            wait_list.erase(std::find(wait_list.begin(), wait_list.end(), root_));
        }
//        std::vector<size_t> nexts = Find_Next(root_);
//        size_t next_ = pool[root_].nexts[pool[root_].visit_time];
//        size_t next_ = nexts[pool[root_].visit_time];
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
//            std::vector<size_t> nexts_ = Find_Next(next_);
//            next_ = nexts_[pool[next_].visit_time];
//            pool[next_].visit_time += 1;

//            if ()

//            std::cout << next_ << std::endl;
            pool[next_].visit_time += 1;
            next_ = pool[next_].nexts[pool[next_].visit_time-1];
//            pool[next_].visit_time += 1;

//            std::cout << next_ << std::endl;
        }
        B += ("L" + std::to_string(next_));
        branch.push_back(next_);
        pool[next_].visit_time += 1;
//        std::cout << B << std::endl;
        Ls.push_back(B);
        branches.push_back(branch);
    }
}