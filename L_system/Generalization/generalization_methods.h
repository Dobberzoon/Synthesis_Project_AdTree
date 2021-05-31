//
// Created by hyde on 5/13/21.
//

#ifndef SYNTHESIS_PROJECT_ADTREE_TEST_METHODS_H
#define SYNTHESIS_PROJECT_ADTREE_TEST_METHODS_H


#include <iostream>
#include <fstream>
#include <easy3d/core/graph.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/types.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/fileio/graph_io.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/algo/remove_duplication.h>
#include <easy3d/util/file_system.h>
#include <typeinfo>
#include "AdTree/skeleton.h"
#include "AdTree/tree_viewer.h"

#include "l_branch_generalization.h"


void get_detail_branches(Skeleton *skl, const std::string &output_folder) {
    Graph& graph = *(const_cast<Graph*>(&skl->get_simplified_skeleton()));
    std::pair<SGraphVertexIterator, SGraphVertexIterator> vi = boost::vertices(graph);
    std::pair<SGraphEdgeIterator, SGraphEdgeIterator> ei = boost::edges(graph);

    std::ofstream my_file(output_folder+"/tiny_details.txt");

    my_file << "Root: " << *(&skl->get_root()) <<std::endl;

    for (SGraphVertexIterator vit = vi.first; vit != vi.second; ++vit) {
        SGraphVertexDescriptor cur_vd = *vit;
        SGraphVertexProp& vp = graph[cur_vd];
//        vp.visited = false;
        my_file << "V id: " << cur_vd << ", degree: " << boost::degree(cur_vd, graph)<< ", parent: " << vp.nParent;
        my_file << "  {" <<std::endl;

        std::pair<Graph::out_edge_iterator, Graph::out_edge_iterator> outei = boost::out_edges(cur_vd, graph);
        for (auto eit = outei.first; eit!=outei.second; ++eit){
            my_file << "-->" << boost::target(*eit, graph) << std::endl;
        }
        my_file << "  }" << std::endl;
    }

    my_file << "=======================" << std::endl;

    for (auto eit = ei.first; eit != ei.second; ++eit) {
        SGraphEdgeDescriptor cur_e = *eit;
        SGraphEdgeProp& e = graph[cur_e];
        my_file << "E id: " << cur_e;
        my_file << std::endl;
    }
    my_file.close();
}


void print_branches(Lbranch lbranch, const std::string &output_folder){
    std::ofstream my_file(output_folder+"/tiny_details.txt");

    for (auto const & mit : lbranch.get_pool()){
        my_file << "node: " << mit.first << " (degree: ";
        my_file << mit.second.degree << "): -->";
        for (auto next:mit.second.nexts){
            my_file << next << " ";
        }
        my_file <<std::endl;
    }

    my_file << "==================================" << std::endl;

    for (auto B:lbranch.get_Ls()){
        my_file << B << std::endl;
        my_file << "-------" << std::endl;
    }
    my_file.close();
}


int l_test(std::vector<std::string>& point_cloud_files, const std::string& output_folder) {

    // - copied from main/batch_reconstruct()
    int count(0);
    for (std::size_t i = 0; i < point_cloud_files.size(); ++i) {
        const std::string &xyz_file = point_cloud_files[i];
        std::cout << "------------- " << i + 1 << "/" << point_cloud_files.size() << " -------------" << std::endl;
        std::cout << "processing xyz_file: " << xyz_file << std::endl;

        if (!easy3d::file_system::is_directory(output_folder)) {
            if (easy3d::file_system::create_directory(output_folder))
                std::cout << "created output directory '" << output_folder << "'" << std::endl;
            else {
                std::cerr << "failed creating output directory" << std::endl;
                return 0;
            }
        }

        // load point_cloud
        easy3d::PointCloud *cloud = easy3d::PointCloudIO::load(xyz_file);
        if (cloud) {
            std::cout << "cloud loaded. num points: " << cloud->n_vertices() << std::endl;

            // compute bbox
            easy3d::Box3 box;
            auto points = cloud->get_vertex_property<easy3d::vec3>("v:point");
            for (auto v : cloud->vertices())
                box.add_point(points[v]);

            // remove duplicated points
            const float threshold = box.diagonal() * 0.001f;
            const auto &points_to_remove = easy3d::RemoveDuplication::apply(cloud, threshold);
            for (auto v : points_to_remove)
                cloud->delete_vertex(v);
            cloud->garbage_collection();
            std::cout << "removed too-close points. num points: " << cloud->n_vertices() << std::endl;
        } else {
            std::cerr << "failed to load point cloud from '" << xyz_file << "'" << std::endl;
            continue;
        }

        auto *mesh = new easy3d::SurfaceMesh;
        const std::string &branch_filename = easy3d::file_system::base_name(cloud->name()) + "_branches.obj";
        mesh->set_name(branch_filename);

        auto *skeleton = new Skeleton();
        bool status = skeleton->reconstruct_branches(cloud, mesh);

    // - copied from main/batch_reconstruct()

        /// L-system initialization
        auto *lsys = new Lsystem();
        lsys->readSkeleton(skeleton);
        // //l_sys->outputLsys(file_system::extension(file_name), file_name);

        lsys->printLsystem();
        std::cout << "--------------------\n" << std::endl;

        if (status) {
//            get_detail_branches(skeleton, output_folder);
            float th_d = 0.1;
            float th_x = 0.00005;
            float th_y = 0.5;
            Lbranch lbranch(lsys, th_d, th_x, th_y);

            lbranch.build_branches();
//            print_branches(lbranch, output_folder);
//            lbranch.print_detail();

            lbranch.lsys_describe_branchnode(lsys);
            std::cout << std::endl;

//            for (auto bnode : lbranch.get_branchnodes()){
//                std::cout << "movement to node " << bnode.node_skel << ": " <<
//                "\n\tforward:  " << bnode.lsys_motion["forward"] << " " <<
//                "\n\trotation: " << bnode.lsys_motion["rotation"] << " " <<
//                "\n\troll:     " << bnode.lsys_motion["roll"] << std::endl;
//            }

            std::cout << "\nF: " << lbranch.get_pool()[3].lsys_motion["forward"] << std::endl;
            std::cout << "\nF: " << lbranch.get_branchnodes()[3].lsys_motion["forward"] << std::endl;
            std::cout << "idx: " << lbranch.get_pool()[3].node_skel << std::endl;

            /// walk all branches node-by-node, compare movement
            int cursor = 0; // current node number on branch
            int max_branch_size = 2;
            bool finished_comparing = false;
            std::map<std::string, std::vector<SGraphVertexDescriptor>> rules;   // rule as string, with list of nodes having it
            /*while (!finished_comparing){
                // collect the node of the current step on all branches
                std::vector<SGraphVertexDescriptor> current_nodes;
                for (auto branch:lbranch.return_branches()){
                    if (branch.size() > cursor && branch.size() <= max_branch_size){
                        current_nodes.push_back(branch[cursor]);
                    }
                }
                std::cout << "\nnumber of nodes in step " << current_nodes.size() << ":" << std::endl;

                // compare
                if (current_nodes.size() >= 2){
                    for (auto nd:current_nodes){
//                        std::cout << nd << std::endl;   // branch.return_pool()[nd].node_skel
                        for (auto nd_comp:current_nodes){
                            if (nd != nd_comp){
                                if (lbranch.return_pool()[nd].lsys_motion["forward"] ==
                                lbranch.return_pool()[nd_comp].lsys_motion["forward"] &&
                                ! lbranch.return_pool()[nd].lsys_motion["forward"].empty()){
                                    rules[lbranch.return_pool()[nd].lsys_motion["forward"]].push_back(nd);
                                    rules[lbranch.return_pool()[nd].lsys_motion["forward"]].push_back(nd_comp);
                                    std::cout << "rule found: " << lbranch.get_pool()[nd].lsys_motion["forward"] << std::endl;
                                }
                            }
                        }
                    }
                    cursor += 1;
                } else {
                    finished_comparing = true;
                }
            }*/

            std::cout << "nr of leaves: " << lbranch.get_leaves().size() << std::endl;

            int steps_to_average = 1;

            std::vector<SGraphVertexDescriptor> current_step = lbranch.get_leaves();
            lbranch.traverse_branch(current_step);

            // list of nexts, average movements (wirtten to nodes?)



            /// rewrite the axiom
            // - walk skeleton once
            // - rewrite lstring to axiom with rules
            // - recursively for nesting, see Lsystem::traverse()

            /*lbranch.Build_Branches();
            for (std::string l:lbranch.Return_Ls()){
                std::cout << l << std::endl;
            }
            std::vector<Skeleton::Branch> branches = skeleton->get_branches_parameters();
            int count_branch = 0;
            Graph graph = skeleton->get_simplified_skeleton();
            SGraphVertexDescriptor rootv = skeleton->get_root();
            SGraphVertexProp& vr = graph[rootv];
            std::pair<SGraphVertexIterator, SGraphVertexIterator> vi = boost::vertices(graph);
            std::pair<SGraphEdgeIterator, SGraphEdgeIterator> es = boost::edges(graph);
            std::copy(es.first, es.second, std::ostream_iterator<SGraphEdgeDescriptor> {std::cout, "\n"});
            std::cout<< "vertices number: " << boost::num_vertices(graph) << ", edges number: " << boost::num_edges(graph) << ", root: " << vr.cVert << std::endl;
            SGraphVertexProp& vstart = graph[*(vi.first+1)];
            std::pair<SGraphAdjacencyIterator, SGraphAdjacencyIterator> adjacencies = boost::adjacent_vertices(rootv, graph);
            std::
            for (SGraphAdjacencyIterator cIter = adjacencies.first; cIter != adjacencies.second; ++cIter){
                std::cout << *cIter << std::endl;
            }

            std::cout << "start of vertices: " << typeid(*(vi.first+1)).name() << std::endl;*/

        } else {
            std::cerr << "failed in reconstructing branches" << std::endl;
            delete cloud;
            delete mesh;
            delete skeleton;
            continue;
        }
    }
    return 1;   // originally count
}

#endif //SYNTHESIS_PROJECT_ADTREE_TEST_METHODS_H