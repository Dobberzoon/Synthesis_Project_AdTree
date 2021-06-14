//
// Created by noortje van der Horst on 02/06/2021.
//

#ifndef L_SYSTEM_LBRANCHGEN_H
#define L_SYSTEM_LBRANCHGEN_H

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
#include <AdTree/L-system.h>
#include "AdTree/skeleton.h"


class Lbranch {
public:
    explicit Lbranch(Lsystem* lsys);

    bool degrees = true;
    Graph graph;                                        // simplified skeleton of graph

    struct BranchNode {
        // Attributes
        unsigned degree;                                // nr. of neighbours
        int visit_time = 0;                             // used for building branches
        easy3d::vec3 cVert;                             // coordinates
        std::vector<size_t> nexts;                      // indices node branches to
        SGraphVertexDescriptor node_skel;               // node in skeleton (boost vertex index)
        std::map<std::string, std::string> lsys_motion; // Lstring description of relative movement towards this node
    };

    bool notleaf(size_t vid);
    void print_detail();
    std::vector<size_t> find_next(size_t vid);
    void build_branches();

    std::vector<std::string> get_Ls() {return Ls; }
    std::map<SGraphVertexDescriptor, BranchNode> get_pool() {return pool; }
    std::vector<BranchNode>& get_branchnodes() {return nodes; }
    std::vector<SGraphVertexDescriptor>& get_leaves() {return leaves; }
    std::vector<std::vector<SGraphVertexDescriptor>>& get_branches() {return branches; }
    std::map<std::string, std::string>& get_rules() {return rules; }

    void average_branch(std::vector<SGraphVertexDescriptor> starting_nodes, int nr_steps, std::string rule_marker);
    void branches_to_lsystem(Lsystem *lsys, std::vector<size_t> starts);


private:
    // Attributes
    std::vector<std::string> Ls;                                // Lsys node chain descriptor
    SGraphVertexDescriptor root;                                // root node (used to be index; size_t)
    std::vector<BranchNode> nodes;                              // custom branch node struct
    std::vector<SGraphVertexDescriptor> leaves;                 // list of all leaf nodes
    std::map<SGraphVertexDescriptor , BranchNode> pool;         // index <--> custom branch node struct
    std::vector<std::vector<SGraphVertexDescriptor> > branches; // list of lists of indexes
    std::map<std::string, std::string> rules;                   // rules map for axiom recursion
};


#endif //L_SYSTEM_LBRANCHGEN_H
