//
// Created by hyde on 6/3/21.
//

#ifndef L_SYSTEM_TEST_GROW_H
#define L_SYSTEM_TEST_GROW_H

#endif //L_SYSTEM_TEST_GROW_H

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

#include "gLsystem.h"

int grow_test(std::vector<std::string>& point_cloud_files, const std::string& output_folder) {

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
//        SurfaceMesh *mesh = new SurfaceMesh;
        const std::string &branch_filename = easy3d::file_system::base_name(cloud->name()) + "_branches.obj";
        mesh->set_name(branch_filename);

        Skeleton *skeleton = new Skeleton();
        bool status = skeleton->reconstruct_branches(cloud, mesh);

        if (status) {
            gLsystem gtest;
            gtest.readSkeleton(skeleton, true);
            gtest.printSth();
        }
    }
    return 1;
}