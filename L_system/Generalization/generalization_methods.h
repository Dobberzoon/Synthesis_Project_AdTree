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

            std::cout << std::endl;

//            for (auto bnode : lbranch.get_branchnodes()){
//                std::cout << "movement to node " << bnode.node_skel << ": " <<
//                "\n\tforward:  " << bnode.lsys_motion["forward"] << " " <<
//                "\n\trotation: " << bnode.lsys_motion["rotation"] << " " <<
//                "\n\troll:     " << bnode.lsys_motion["roll"] << " " <<
//                "\n\tnesting:  " << bnode.lsys_motion["nesting"] << std::endl;
//            }

            std::cout << "nr of leaves: " << lbranch.get_leaves().size() << std::endl;

            /// write rules for averaged tips of branches
            int steps_to_average = 2;
            std::string rule_marker = "X";

            std::vector<SGraphVertexDescriptor> current_step = lbranch.get_leaves();
            lbranch.average_branch(current_step, steps_to_average, rule_marker);

            /// write rules and axiom to L-system
            std::vector<size_t> rt;
            rt.push_back(lsys->get_root());

            // clear axiom before writing with rules
            lsys->axiom = "";
            lsys->rules = lbranch.get_rules();
            lbranch.branches_to_lsystem(lsys, rt);
            lsys->printLsystem();

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