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

//    std::string my_file_location = output_folder+"/tiny.json";
    std::ofstream my_file(output_folder+"/tiny_details.txt");

    my_file << "Root: " << *(&skl->get_root()) <<std::endl;

    for (SGraphVertexIterator vit = vi.first; vit != vi.second; ++vit) {
        SGraphVertexDescriptor cur_vd = *vit;
        SGraphVertexProp& vp = graph[cur_vd];
//        vp.visited = false;
//        std::cout << "id: " << cur_vd << ", (" << vp.cVert  << "), parent:" << vp.nParent << std::endl;
        my_file << "V id: " << cur_vd << ", degree: " << boost::degree(cur_vd, graph)<< ", parent: " << vp.nParent;
        my_file << "  {" <<std::endl;
//        Graph::out_edge_iterator eit, eend;
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
//        for (auto i:e.vecPoints){
//            my_file << i << " ";
//        }
        my_file << std::endl;
    }

    my_file.close();

}

void print_detail(Lbranch lbranch, const std::string &output_folder){
    std::ofstream my_file(output_folder+"/tiny_details.txt");

    for (auto const & mit : lbranch.return_pool()){
        my_file << "node: " << mit.first << " (degree: ";
        my_file << mit.second.degree << "): -->";
        for (auto next:mit.second.nexts){
            my_file << next << " ";
        }
        my_file <<std::endl;
    }

    my_file << "==================================" << std::endl;

    for (auto B:lbranch.return_Ls()){
        my_file << B << std::endl;
        my_file << "-------" << std::endl;
    }
    my_file.close();
}



int l_test(std::vector<std::string>& point_cloud_files, const std::string& output_folder) {

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

        easy3d::SurfaceMesh *mesh = new easy3d::SurfaceMesh;
        const std::string &branch_filename = easy3d::file_system::base_name(cloud->name()) + "_branches.obj";
        mesh->set_name(branch_filename);

        Skeleton *skeleton = new Skeleton();
        bool status = skeleton->reconstruct_branches(cloud, mesh);

        // todo: this is where the l-system normally goes
        Lsystem *lsys = new Lsystem();
        lsys->readSkeleton(skeleton);
        // //l_sys->outputLsys(file_system::extension(file_name), file_name);

        lsys->printLsystem();

        if (status) {
//            std::cerr << "failed in reconstructing branches" << std::endl;
//            delete cloud;
//            delete mesh;
//            delete skeleton;
//            continue;
//            get_detail_branches(skeleton, output_folder);
            float th_d = 0.1;
            float th_x = 0.00005;
            float th_y = 0.5;
            Lbranch lbranch(skeleton, th_d, th_x, th_y);
//            std::cout << "here" << std::endl;
            lbranch.build_branches();
            print_detail(lbranch, output_folder);
            lbranch.print_detail();
//            lbranch.Build_Branches();
//            for (std::string l:lbranch.Return_Ls()){
//                std::cout << l << std::endl;
//            }
//            std::vector<Skeleton::Branch> branches = skeleton->get_branches_parameters();
//            int count_branch = 0;
//            Graph graph = skeleton->get_simplified_skeleton();
//            SGraphVertexDescriptor rootv = skeleton->get_root();
//            SGraphVertexProp& vr = graph[rootv];
//            std::pair<SGraphVertexIterator, SGraphVertexIterator> vi = boost::vertices(graph);
//            std::pair<SGraphEdgeIterator, SGraphEdgeIterator> es = boost::edges(graph);
//            std::copy(es.first, es.second, std::ostream_iterator<SGraphEdgeDescriptor> {std::cout, "\n"});
//            std::cout<< "vertices number: " << boost::num_vertices(graph) << ", edges number: " << boost::num_edges(graph) << ", root: " << vr.cVert << std::endl;
//            SGraphVertexProp& vstart = graph[*(vi.first+1)];
//            std::pair<SGraphAdjacencyIterator, SGraphAdjacencyIterator> adjacencies = boost::adjacent_vertices(rootv, graph);
//            std::
//            for (SGraphAdjacencyIterator cIter = adjacencies.first; cIter != adjacencies.second; ++cIter){
//                std::cout << *cIter << std::endl;
//            }

//            std::cout << "start of vertices: " << typeid(*(vi.first+1)).name() << std::endl;

        }
    }
    return 1;
}

#endif //SYNTHESIS_PROJECT_ADTREE_TEST_METHODS_H