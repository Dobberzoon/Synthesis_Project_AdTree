//
// Created by hyde on 5/13/21.
//

#ifndef SYNTHESIS_PROJECT_ADTREE_L_BRANCH_H
#define SYNTHESIS_PROJECT_ADTREE_L_BRANCH_H


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

//class Lbranch {
//public:
//    Lbranch(Lsystem* lsystem, float th_d, float th_x, float th_y);
//
//    bool degrees = true;
//    Graph graph;                                        // simplified skeleton of graph
//
//    struct BranchNode {
//        // Attributes
//        unsigned degree;                                // nr. of neighbours
//        int visit_time = 0;                             // ?? (used for building branches)
//        easy3d::vec3 cVert;                             // coordinates
//        std::vector<size_t> nexts;                      // indices node branches to
//        SGraphVertexDescriptor node_skel;               // node in skeleton (boost vertex index)
//        // Lstring description of movement towards this node (from its parent)
//        std::map<std::string, std::string> lsys_motion ;
//
//    };
//
//    bool notleaf(size_t vid);
//    void print_detail();
//    std::vector<size_t> find_next(size_t vid);
//    void build_branches();
//
//    std::vector<std::string> get_Ls() {return Ls; }
//    std::map<SGraphVertexDescriptor, BranchNode> get_pool() {return pool; }
//    std::vector<BranchNode>& get_branchnodes() {return nodes; }
//    std::vector<SGraphVertexDescriptor>& get_leaves() {return leaves; }
//    std::vector<std::vector<SGraphVertexDescriptor>>& get_branches() {return branches; }
//    std::map<std::string, std::string>& get_rules() {return rules; }
//
//    void average_branch(std::vector<SGraphVertexDescriptor> starting_nodes, int nr_steps, std::string rule_marker);
//    void branches_to_lsystem(Lsystem *lsys, std::vector<size_t> starts);
//
//
//
//private:
//    // Attributes
//    std::vector<std::string> Ls;                                // Lsys node chain descriptor
////    Skeleton* skl;                                            // ...
//    SGraphVertexDescriptor root;                                // root node (used to be index; size_t)
////    std::vector<size_t> vs;                                   // vertices
//    std::vector<BranchNode> nodes;                              // custom branch node struct
//    std::vector<SGraphVertexDescriptor> leaves;                 // list of all leaf nodes
//    std::map<SGraphVertexDescriptor , BranchNode> pool;         // index <--> custom branch node struct
//    std::vector<std::vector<SGraphVertexDescriptor> > branches; // list of lists of indexes
//    std::map<std::string, std::string> rules;                   // rules map for axiom recursion
//    float th_d;                                                 // ??
//    float th_x;                                                 // ??
//    float th_y;                                                 // ??
//};
//
//
//// initialize graph & node organisation
//Lbranch::Lbranch(Lsystem *lsys, float th_d, float th_x, float th_y) {
////    skl = skeleton;
//    graph = lsys->graph_lsys;
//    root = lsys->get_root();
//    degrees = lsys->isDegrees();
//    this->th_d = th_d; this->th_x=th_x; this->th_y = th_y;
//    std::pair<SGraphVertexIterator, SGraphVertexIterator> vi = boost::vertices(graph);
//    for (auto vit = vi.first; vit != vi.second; ++vit){
//        if (boost::degree(*vit, graph)!=0) {
////            vs.push_back(*vit);
//            BranchNode temp;
//            temp.degree = boost::degree(*vit, graph);
//            temp.cVert = graph[*vit].cVert;
//            temp.node_skel = *vit;
//            temp.lsys_motion = graph[*vit].lstring;
//            if(notleaf(*vit)) {
//                temp.nexts = find_next(*vit);
//            } else {
//                leaves.push_back(*vit);
//            }
//            nodes.push_back(temp);
//            pool.insert(std::make_pair(*vit, temp));
//        }
//    }
//}
//
//
//bool Lbranch::notleaf(size_t vid) {
//    if (graph[vid].nParent != vid && boost::degree(vid, graph)==1) return false;
//    return true;
//}
//
//
//std::vector<size_t> Lbranch::find_next(size_t vid) {
//    std::pair<Graph::out_edge_iterator, Graph::out_edge_iterator> outei = boost::out_edges(vid, graph);
//    std::vector<size_t> nexts_;
//    for (auto eit = outei.first; eit!=outei.second; ++eit){
//        if (boost::target(*eit, graph)!=graph[vid].nParent) nexts_.push_back(boost::target(*eit, graph));
//    }
//    return nexts_;
//}
//
//
//void Lbranch::print_detail() {
//    for (auto B:Ls){
//        std::cout << B << std::endl;
//    }
//    for (auto b:branches){
//        for (auto n:b){
//            std::cout << n << " ";
//        }
//        std::cout << std::endl;
//    }
//}
//
//
//// make visit_time, Ls & branches
//void Lbranch::build_branches() {
//    std::vector<size_t> wait_list;
//    wait_list.push_back(root);
//    while (!wait_list.empty()){
//        size_t root_ = wait_list[0];
//        std::string B;
//        std::vector<size_t> branch;
//        if (root_ == root) {
//            B += "R --> ";
//        }
//        else B+= "N"+std::to_string(root_)+"_"+std::to_string(pool[root_].degree-1-pool[root_].visit_time)+" --> ";
//        branch.push_back(root_);
//
//        size_t next_ = pool[root_].nexts[pool[root_].visit_time];
//        pool[root_].visit_time+=1;
//        if (pool[root_].degree-1 <= pool[root_].visit_time) {
//            wait_list.erase(std::find(wait_list.begin(), wait_list.end(), root_));
//        }
//        while (notleaf(next_)){
//            int ns = pool[next_].degree-1 - pool[next_].visit_time;
//            if (ns>1){
//                B+="(";
//                for (int i=pool[next_].visit_time; i<ns; ++i){
//                    B+= "N"+std::to_string(next_)+"_"+std::to_string(i+1)+" ";
//                }
//                B+=") --> ";
//            }
//            else B+= "N"+std::to_string(next_)+" --> ";
//            branch.push_back(next_);
//            if (pool[next_].degree-2 > pool[next_].visit_time) {
//                wait_list.push_back(next_);
//            }
//
//            pool[next_].visit_time += 1;
//            next_ = pool[next_].nexts[pool[next_].visit_time-1];
//        }
//        B += ("L" + std::to_string(next_));
//        branch.push_back(next_);
//        pool[next_].visit_time += 1;
//        Ls.push_back(B);
//        branches.push_back(branch);
//    }
//}
//
//
//void Lbranch::average_branch(std::vector<SGraphVertexDescriptor> starting_nodes, int nr_steps, std::string rule_marker){
//    std::map<std::string, float> averages = {{"forward",  0},
//                                             {"rotation", 0},
//                                             {"roll",     0}};
//    std::vector<SGraphVertexDescriptor> next_vertices;
//
//    float forward_total = 0;
//    float rotation_total = 0;
//    float roll_total = 0;
//
//    // find average & nexts
//    for (SGraphVertexDescriptor nd:starting_nodes) {
//        /// parents of branch tips
//        next_vertices.push_back(graph[nd].nParent);
//
//        /// forward
//        std::string line_f = get_pool()[nd].lsys_motion["forward"];
//        // line is not an empty string and is overwritten
//        if (line_f.size() > 2) {
//            std::string sValue;
//            // skip "F" and "("
//            for (int i = 2; i < line_f.size(); ++i) {
//                if (line_f[i] != ')') {
//                    sValue += line_f[i];
//                }
//            }
//            float value = std::stod(sValue);
//            forward_total += value;
//        }
//
//        /// rotation
//        std::string line_rot = pool[nd].lsys_motion["rotation"];
//        // line is not an empty string and is overwritten
//        if (line_rot.size() > 2) {
//            std::string sValue;
//            int i = 0;
//            bool negative_rotation = false;
//            while (line_rot[i] != ')') {
//                if (line_rot[i] == '-') {
//                    negative_rotation = true;
//                } else if (line_rot[i] != '(' && line_rot[i] != '+') {
//                    sValue += line_rot[i];
//                }
//                i++;
//            }
//            float value = std::stod(sValue);
//            // to make sure negative rotation gets read the same as positive rotation
//            if (negative_rotation) {
//                if (degrees) {
//                    value = 360 - value;
//                } else {
//                    value = 2 * M_PI - value;
//                }
//            }
//            rotation_total += value;
//        }
//
//        /// roll
//        std::string line_roll = pool[nd].lsys_motion["roll"];
//        // line is not an empty string and is overwritten
//        if (line_roll.size() > 2) {
//            std::string sValue;
//            int i = 0;
//            bool negative_roll = false;
//            while (line_roll[i] != ')') {
//                if (line_roll[i] == '<') {
//                    negative_roll = true;
//                } else if (line_roll[i] != '(' && line_roll[i] != '>') {
//                    sValue += line_roll[i];
//                }
//                i++;
//            }
//            float value = std::stod(sValue);
//            // to make sure negative roll gets read the same as positive roll
//            if (negative_roll) {
//                if (degrees) {
//                    value = 360 - value;
//                } else {
//                    value = 2 * M_PI - value;
//                }
//            }
//            roll_total += value;
//        }
//    }
//
//    float forward_average = forward_total / starting_nodes.size();
//    float rotation_average = rotation_total / starting_nodes.size();
//    float roll_average = roll_total / starting_nodes.size();
//
//    // todo: add configurable accuracy to float conversions
//
//    std::string forward_rule = "F(" + std::to_string(forward_average) + ")";
//    std::string rotation_rule = "+(" + std::to_string(rotation_average) + ")";
//    std::string roll_rule = ">(" + std::to_string(roll_average) + ")";
//
//    // if still more than 1 step left, continue, mark as to be deleted part of axiom
//    if (nr_steps - 1 > 0){
//        // write average movement to start of current rule
//        std::string current_rule = rules[rule_marker];
//        rules[rule_marker] = rotation_rule + roll_rule + forward_rule + current_rule;
//        // mark all current step nodes to belong to not-start of rule
//        for (SGraphVertexDescriptor nd:starting_nodes) {
//            graph[nd].lstring["forward"] = rule_marker + "*";
//        }
//        average_branch(next_vertices, nr_steps -1, rule_marker);
//    }
//    // if this was the last step: rule marker
//    else if (nr_steps - 1 == 0){
//        // write average movement to start of current rule
//        std::string current_rule = rules[rule_marker];
//        rules[rule_marker] = rotation_rule + roll_rule + forward_rule + current_rule;
//        // mark all current step nodes to belong to start of rule
//        for (SGraphVertexDescriptor nd:starting_nodes){
//            graph[nd].lstring["forward"] = rule_marker;
//        }
//    }
//}
//
//
//void Lbranch::branches_to_lsystem(Lsystem *lsys, std::vector<size_t> starts){
//    // traverse the given nodes
//    for (auto nd:starts){
//        bool rule_found = false;
//        // check for rules
//        for (auto rule:rules){
//            std::string forward_cur = graph[nd].lstring["forward"];
//
//            // start node of rule
//            if (forward_cur == rule.first) {
//                rule_found = true;
//
//                // write nesting & rule marker
//                int back_nest = 0;
//                for (auto nest:graph[nd].lstring["nesting"]) {
//                    if (nest == '[') {
//                        lsys->axiom += '[';
//                    }
//                    if (nest == ']') {
//                        back_nest++;
//                    }
//                }
//                lsys->axiom += rule.first;
//
//                for (int i = 0; i < back_nest; ++i) {
//                    lsys->axiom += ']';
//                }
//            }
//
//            // continuation node of rule
//            if (forward_cur == rule.first + '*'){
//                rule_found = true;
//                // just write nesting
//                lsys->axiom += graph[nd].lstring["nesting"];
//                // todo: remove useless nesting markers
//            }
//            // todo: write rules to lsystem rules
//        }
//
//        // if no rules, write all movement
//        if (!rule_found){
//            int back_nest = 0;
//            for (auto nest:graph[nd].lstring["nesting"]){
//                if (nest == '['){
//                    lsys->axiom += '[';
//                }
//                if (nest == ']'){
//                    back_nest++;
//                }
//            }
//            lsys->axiom += graph[nd].lstring["rotation"] +
//                           graph[nd].lstring["roll"] +
//                           graph[nd].lstring["forward"];
//
//            for (int i = 0; i < back_nest; ++i) {
//                lsys->axiom += ']';
//            }
//        }
//        // go down next step in traversal
//        branches_to_lsystem(lsys, pool[nd].nexts);
//    }
//}


#endif //SYNTHESIS_PROJECT_ADTREE_L_BRANCH_H