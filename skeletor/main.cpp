
#include <string>
#include <vector>
#include <sstream>
#include <unordered_map>
#include <math.h>
#include <list>
#include <map>
#include "Point.h"
#include "Rows.h"
#include <cstdlib>
#include <iostream>
#include <cmath>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Polyhedron_3.h>
//#include <CGAL/Surface_mesh.h>
//#include <CGAL/convex_hull_3.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <any>
//#include <boost/uuid/uuid.hpp>
//#include <boost/uuid/uuid_generators.hpp>
//#include <boost/uuid/uuid_io.hpp>
using json = nlohmann::json;


using namespace std;



int read_object(std::string  input, std::map<int, Point > &Vertices,
                std::map<int, std::vector<int>> &Branches, std::map<int, float> &BranchRadii,
                std::map<int, int> &EndBranch, float maxradius)
{
    std::cout << "Reading file: " << input << std::endl;
    std::ifstream infile(input.c_str(), std::ifstream::in);
    if (!infile)
    {
        std::cerr << "Input file not found.\n";
        return false;
    }
    std::string cursor;
    std::string line = "";

    std::getline(infile, line);
    std::getline(infile, line);

    while (line != "")
    {
        
        std::istringstream linestream(line);

        linestream >> cursor;

        float x1,y1,z1, r, x2, y2, z2;
        int IDB, ID1, ID2, e;
        IDB = std::stoi(cursor);
        linestream >> cursor;

        ID1 = std::stoi(cursor);
        linestream >> cursor;

        ID2 = std::stoi(cursor);
        linestream >> cursor;

        x1 = std::stof(cursor);
        linestream >> cursor;

        y1 = std::stof(cursor);
        linestream >> cursor;
        z1 = std::stof(cursor);
        linestream >> cursor;

        x2 = std::stof(cursor);
        linestream >> cursor;
        y2 = std::stof(cursor);
        linestream >> cursor;


        z2 = std::stof(cursor);
        linestream >> cursor;
        r = std::stof(cursor);
        linestream >> cursor;
        e = std::stof(cursor);

        auto p1 = Point(x1,y1,z1);
        auto p2 = Point(x2,y2,z2);
        Vertices[ID1] = p1;
        Vertices[ID2] = p2;
        std::vector vectorBranch{ID1, ID2};
        Branches[IDB] = vectorBranch;
        BranchRadii[IDB] = r;
        EndBranch[IDB] = e;

        if (r > maxradius){
            maxradius = r;
        }
        std::getline(infile, line);


    }

    return 0;
}





int writeJSON( std::map<int, Point> &Vertices, std::map<int, std::vector<int>> &Branches,
                std::map<int, float> &BranchRadii, std::map<int, int> &EndBranch, std::string output_file, float maxradius){
    json final;
    final["type"] = "CityJSON";
    final["version"] = "1.0";


    //creating object with all vertices
    std::cout<<"creating object with all vertices "<<std::endl;
    json vertices= json::array();
    
    for (int i=0; i < Vertices.size(); i++){
        std::vector<float> array1{Vertices[i].x, Vertices[i].y, Vertices[i].z};
        json subvertices= json::array();
        for (int j = 0; j < array1.size(); j++){
               subvertices.emplace_back(array1[j]);
        }
    vertices.emplace_back(subvertices);
    }

    //creating boundaries
    std::cout<<"creating boundaries"<<std::endl;

    json boundaries= json::array();
    for (int i=0; i < Branches.size(); i++){
        json branch = json::array();
        branch.emplace_back(Branches[i][0]);
        branch.emplace_back(Branches[i][1]);
        boundaries.emplace_back(branch);
    }


    //create semantic classes
    std::cout<<"create semantic classes"<<std::endl;
    json semantics;
    json types;
    for (int i=1; i < 11; i++){
        float radius = (maxradius*i)/11;
        json classtemp { { "class",i, "radius",radius} };
        types.emplace_back(classtemp);
    }
    json typestemp{ {"class", 11}, {"radius", maxradius / 12 } };
    types.emplace_back(typestemp); // cleass for ends of branches

    //assigning branches to classes and putting them into values array
    std::cout<<"assigning branches to classes and putting them into values array"<<std::endl;
    json values;
    for (int i=0; i < BranchRadii.size(); i++){
        for (int j=1; j <= 11; j++){
            if ((BranchRadii[i] > ((maxradius*j-1)/11)) && (BranchRadii[i] <= (maxradius*(j)/11))){
                values[i] = j;
            }
                // if( ((BranchRadii[i] < ((maxradius)/11)))){
                //      values[i] = 1;
                // }
            }
        }
            


        
        
    

    //assign ends of branches
    std::cout<<"assign ends of branches"<<std::endl;

    for (int i=1; i < EndBranch.size(); i++){
        if (EndBranch[i] == 1){
            values[i] = 11;
        }
    }

    //write semantics
    std::cout<<"write semantics"<<std::endl;

    semantics["types"] = types;
    semantics["values"] = values;

    //creating full geometry
    std::cout<<"creating full geometry"<<std::endl;
    json geometry {{"type", "MultilineString"}, {"lod", 2},
                   {"boundaries", boundaries}, {"semantics", semantics} };
    final["geometry"] = geometry;



    std::cout<<"Writing vertices.."<<std::endl;
    final["vertices"] = vertices;
    std::cout<<"vertices written"<<std::endl;

    std:: ofstream MyFile(output_file);
    std::cout<<"dumping file..."<<std::endl;
    MyFile << final.dump(4);
    MyFile.close();
    std::cout<<"file closed"<<std::endl;
    return 0;
}

int main()
{
    std::map<int, Point> Vertices;
    std::map<int, std::vector<int>> Branches;
    std::map<int, float> BranchRadii;
    std::map<int, int> EndBranch;
    float maxradius = 0.01;
    const char *file_in = "data.txt";
    std:: string  input =  file_in;
    input = "../" + input;
    const char *file_out = "Treesfile.txt";

    std:: string  output =  file_out;
    output = "../" + output;
    read_object(input, Vertices, Branches, BranchRadii, EndBranch, maxradius);
    std::cout<<" reader done " <<std::endl;
    writeJSON(Vertices, Branches, BranchRadii, EndBranch, output, maxradius);
    std::cout<<"return"<<std::endl;
    return 0;

}