//
// Created by jasper van der vaart on 18/05/2021.
//
#ifndef TURTLE_TURTLE_H
#define TURTLE_TURTLE_H

#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <easy3d/core/vec.h>
#include <easy3d/core/mat.h>
#include <boost/graph/adjacency_list.hpp>

#include "nlohmann/json.hpp"

struct SGraphVertexProp
{
    easy3d::Vec<3, double>  cVert;
    std::size_t nParent{};
    double lengthOfSubtree{};

    double radius{}; // used only by the smoothed skeleton
    bool   visited{};
};

struct SGraphEdgeProp
{
    double nWeight;
    double nRadius;
    std::vector<int> vecPoints;
};

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SGraphVertexProp, SGraphEdgeProp > Graph;

class Turtle {
public:
    /// default constructor ///
    explicit Turtle (double x = 0, double y = 0, double z = 0);

    /// construct turtle based on another turtle location ///
    explicit Turtle(easy3d::Vec<3, double> p);

    /// construct a memoryless turtle based on another turtle ///
    explicit Turtle(Turtle const &other);

    /// output the current location of the turtle ///
    void printLocation() const;

    /// initializer override from crooked stems (do not use while walking)///
    void setRotation(double angle, double roll = 0);

    /// debug mode prints the location of the turtle after every move ///
    void setDebug(bool b);

    /// set the angle and roll functions to function with degrees instead of radians///
    void set2Degrees();

    /// return the points that were internalized ///
    auto getStoredPoints();

    /// return the stored edges that were internalized ///
    std::vector<std::vector<unsigned int>> getStoredEdges();

    Graph getGraph();

    /// store internalized points to file ///
    void writeToXYZ(const std::string& fileName);

    /// store internalized points and edges to file ///
    void writeToPly(const std::string& fileName);

    /// read json file ///
    void readFile(const std::string& path);

private:
    // location is a 3d coordinate
    easy3d::Vec<3, double> loc;

    // plane is a 2d plane in a 3d space
    easy3d::Mat<3,3, double> plane;

    // collection of stored edges
    std::vector<std::vector<unsigned int>> storedEdges;

    // graph
    Graph graph;

    // default variables
    double fValue = 5;
    double rotateValue = 10;
    double rollValue = 10;

    // prints location at every step
    bool debug = false;

    // communicate with user
    bool com = true;

    // set turtle to degrees
    bool deg = false;

    /// step forward ///
    void stepForward(double distance);

    /// rotate ///
    void rotatePlane(double angle);

    /// roll ///
    void rollPlane(double rollAngle);

    /// internalize current location ///
    void storeLoc(unsigned int parent = -1);

    /// bind the provided default values ///
    void setDefaultValues(const nlohmann::json& d);

    /// remove data that is not written from read string ///
    static std::string cleanLine(std::string line);

    /// translate the complex axiom to a "simple" line ///
    static std::string translateLine(const nlohmann::json& axiom, const nlohmann::json& rules, nlohmann::json r);

    /// translate the "simple" line to 3d points ///
    void readLine(std::string line);

};


#endif TURTLE_TURTLE_H
