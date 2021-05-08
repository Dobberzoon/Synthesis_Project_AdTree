//
// Created by noort on 06/05/2021.
//

#include "L-system.h"

#include "skeleton.h"
#include "cylinder.h"

#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/random.h>
#include <easy3d/core/principal_axes.h>
#include <3rd_party/tetgen/tetgen.h>

#include <iostream>
#include <algorithm>


using namespace boost;
using namespace easy3d;


Lsystem::Lsystem()
    {
        string_ = "";
        axiom_ = "";
        graph_.clear();
    }

void Lsystem::printLsystem() {
    std::cout << "printing L-system..." << std::endl;
    std::cout << "string: " << string_ << std::endl;
    std::cout << "axiom:  " << axiom_ << std::endl;
}

void Lsystem::readSkeleton(Skeleton* skel){
    std::cout << "nr. vertices of smooth skeleton: " << num_vertices(skel->get_smoothed_skeleton()) << std::endl;
    std::cout << "nr. vertices of simplified skeleton: " << num_vertices(skel->get_simplified_skeleton()) << std::endl;
    printLsystem();

    // get paths
    std::vector<Path> pathList;
    makePathsLsystem(skel, pathList);

    std::cout << "number of paths: " << pathList.size() << std::endl;

    // for each path
    for (std::size_t n_path = 0; n_path < pathList.size(); ++n_path) {
        if (n_path != 0){
            string_ += "]";
        }

        Path currentPath = pathList[n_path];
        std::vector<vec3> interpolatedPoints;

        std::cout << "\n---------- new subpath ----------" << std::endl;
        std::cout << "subpath length: " << currentPath.size() << std::endl;
        std::cout << "root subpath:   " << skel->get_simplified_skeleton()[currentPath[0]].cVert << std::endl;

        for (std::size_t n_node = 0; n_node < currentPath.size() - 1; ++n_node) {
            SGraphVertexDescriptor sourceV = currentPath[n_node];
            SGraphVertexDescriptor targetV = currentPath[n_node + 1];
            vec3 pSource = skel->get_simplified_skeleton()[sourceV].cVert;
            vec3 pTarget = skel->get_simplified_skeleton()[targetV].cVert;
            float branchlength = easy3d::distance(pSource, pTarget);

            std::cout << "source: " << skel->get_simplified_skeleton()[sourceV].cVert
                      << " --> target: " << skel->get_simplified_skeleton()[targetV].cVert
                      << std::endl;

            std::cout << "branch length: " << branchlength << std::endl;

            // compute the tangents
            vec3 tangentOfSorce;
            vec3 tangentOfTarget;
            // if the source vertex is the root
            if (sourceV == skel->get_simplified_skeleton()[sourceV].nParent) {
                tangentOfSorce = (pTarget - pSource).normalize();

                string_ += std::to_string(sourceV);
            }
            else
            {
                SGraphVertexDescriptor parentOfSource = skel->get_simplified_skeleton()[sourceV].nParent;
                tangentOfSorce = (pTarget - skel->get_simplified_skeleton()[parentOfSource].cVert).normalize();

                if (out_degree(sourceV, skel->get_simplified_skeleton()) > 2){
                    string_ += "[";
                }

            }
            // if the target vertex is leaf
            if ((out_degree(targetV, skel->get_simplified_skeleton()) == 1)
            && (targetV != skel->get_simplified_skeleton()[targetV].nParent)){
                tangentOfTarget = (pTarget - pSource).normalize();

                string_ += std::to_string(targetV);
                string_ += "]";
            }
            else
            {
                SGraphVertexDescriptor childOfTarget = currentPath[n_node + 2];
                tangentOfTarget = (skel->get_simplified_skeleton()[childOfTarget].cVert - pSource).normalize();

                string_ += std::to_string(targetV);
            }

            std::cout << "source tangent: " << tangentOfSorce
                      << " | target tangent: " << tangentOfTarget
                      << std::endl;

            tangentOfSorce *= branchlength;
            tangentOfTarget *= branchlength;
        }
    }
    std::cout << "writing L-system: done" << std::endl;
    printLsystem();
}


// copied the path making, but for the smooth one
void Lsystem::makePathsLsystem(Skeleton* skel, std::vector<Path> &pathList) {
    pathList.clear();
    Path currentPath;
    int cursor = 0;
    //insert the root vertex to the current path
    currentPath.push_back(skel->get_root());
    pathList.push_back(currentPath);
    //retrieve the path list
    while (cursor < pathList.size())
    {
        currentPath = pathList[cursor];
        SGraphVertexDescriptor endV = currentPath.back();
        // if the current path has reached the leaf
        if ((out_degree(endV, skel->get_simplified_skeleton()) == 1) && (endV != skel->get_simplified_skeleton()[endV].nParent))
            cursor++;
        else
        {
            //find the fatest child vertex
            double maxR = -1;
            int isUsed = -1;
            SGraphVertexDescriptor fatestChild;
            std::vector<SGraphVertexDescriptor> notFastestChildren;
            std::pair<SGraphAdjacencyIterator, SGraphAdjacencyIterator> adjacencies = adjacent_vertices(endV, skel->get_simplified_skeleton());
            for (SGraphAdjacencyIterator cIter = adjacencies.first; cIter != adjacencies.second; ++cIter)
            {
                if (*cIter != skel->get_simplified_skeleton()[endV].nParent)
                {
                    SGraphEdgeDescriptor currentE = edge(endV, *cIter, skel->get_simplified_skeleton()).first;
                    double radius = skel->get_simplified_skeleton()[currentE].nRadius;
                    if (maxR < radius)
                    {
                        maxR = radius;
                        if (isUsed > -1)
                            notFastestChildren.push_back(fatestChild);
                        else
                            isUsed = 0;
                        fatestChild = *cIter;
                    }
                    else
                        notFastestChildren.push_back(*cIter);
                }
            }
            // organize children vertices into a new path
            for (int nChild = 0; nChild < notFastestChildren.size(); ++nChild)
            {
                Path newPath;
                newPath.push_back(endV);
                newPath.push_back(notFastestChildren[nChild]);
                pathList.push_back(newPath);
            }
            //put the fatest children into the path list
            pathList[cursor].push_back(fatestChild);
        }
    }

    return;
}


void Lsystem::traverseLsystem(Skeleton *skel) {
    std::cout << "nr. vertices of simplified skeleton: " << num_vertices(skel->get_simplified_skeleton()) << std::endl;
    printLsystem();

    SGraphVertexDescriptor root = skel->get_root();
    traverse(root, skel);

    std::cout << "writing L-system: done" << std::endl;
    printLsystem();
}


void Lsystem::traverse(SGraphVertexDescriptor startV, Skeleton *skel){
    vec3 start_coords = skel->get_simplified_skeleton()[startV].cVert;
    string_ += std::to_string(startV);

    std::cout << "\n---------- new branch ----------" << std::endl;
    std::cout << "starting node: " << startV << std::endl;

    SGraphVertexDescriptor nextV;
    std::vector<SGraphVertexDescriptor> slower_children;

    /* if leaf: write & stop
     else:
         find fastest child: -> nextV
         find other children: -> slower_children

         tangent startV <> nextV
         length edge startV <> nextV

         if degree > 2:
             begin branch: "["

         write nextV
         traverse nextV

         if degree > 2:
             end branch: "]"

         if degree > 2:
             for each child in slower children:
                 begin branch: "["
                 traverse child
                 end branch: "]"

     output: written string_*/

    /// node is leaf
    if ((out_degree(startV, skel->get_simplified_skeleton()) == 1)
        && (startV != skel->get_simplified_skeleton()[startV].nParent)) {

        std::cout << "leaf found: " << startV << std::endl;
    }
    else{
        /// find children of start node
        double maxR = -1;
        int isUsed = -1;

        std::pair<SGraphAdjacencyIterator, SGraphAdjacencyIterator> adjacencies =
                adjacent_vertices(startV, skel->get_simplified_skeleton());

        for (SGraphAdjacencyIterator cIter = adjacencies.first; cIter != adjacencies.second; ++cIter)
        {
            // exclude parent
            if (*cIter != skel->get_simplified_skeleton()[startV].nParent)
            {
                std::cout << "child found: " << *cIter << std::endl;
                SGraphEdgeDescriptor currentE = edge(startV, *cIter, skel->get_simplified_skeleton()).first;
                double radius = skel->get_simplified_skeleton()[currentE].nRadius;
                if (maxR < radius)
                {
                    maxR = radius;
                    if (isUsed > -1)
                        slower_children.push_back(nextV);
                    else
                        isUsed = 0;
                    nextV = *cIter;
                }
                else
                    slower_children.push_back(*cIter);
            }
        }


        std::cout << "fastest child: " << nextV << std::endl;
        std::cout << "slower children: " ;

        for (int nChild = 0; nChild < slower_children.size(); ++nChild){
            std::cout << slower_children[nChild] << " ";
        }

        /// start node has one child: straight segment
        if (out_degree(startV, skel->get_simplified_skeleton()) == 1){
            traverse(nextV, skel);
        }
        /// start node has multiple children: beginning of 2 or more branches
        else{
            string_ += "[";
            traverse(nextV, skel);
            string_ += "]";

            for (int nChild = 0; nChild < slower_children.size(); ++nChild){
                string_ += "[";
                traverse(slower_children[nChild], skel);
                string_ += "]";
            }
        }
    }
}