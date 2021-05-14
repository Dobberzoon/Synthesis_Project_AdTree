#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <easy3d/core/vec.h>
#include <easy3d/core/mat.h>
#include <boost/graph/adjacency_list.hpp>

#include "3rd_party/nlohmann/json.hpp"

class Turtle {
public:
    /// default constructor ///
    explicit Turtle (double x = 0, double y =0, double z =0) {
        loc = {x, y, z};
        plane.set_row(0, easy3d::Vec<3, double>{1,0,0});
        plane.set_row(1, easy3d::Vec<3, double>{0,1,0});
        plane.set_row(2, easy3d::Vec<3, double>{0,0,1});
    }

    /// construct turtle based on another turtle location ///
    explicit Turtle (easy3d::Vec<3, double> p){
        loc = p;
        plane.set_row(0, easy3d::Vec<3, double>{1,0,0});
        plane.set_row(1, easy3d::Vec<3, double>{0,1,0});
        plane.set_row(2, easy3d::Vec<3, double>{0,0,1});
    }

    /// construct a turtle based on another turtle ///
    Turtle (Turtle const &other){
        loc = other.loc;
        plane = other.plane;
        debug = other.debug;
        fValue = other.fValue;
        rollValue = other.rollValue;
        rotateValue = other.rotateValue;
    }

    /// output the current location of the turtle ///
    void printLocation() const{
        std::cout <<"current location of turtle is: (";
        std::cout << loc.x << ", " << loc.y << ", " << loc.z << ")" << std::endl;
    }

    /// initializer override from crooked stems (do not use while walking)///
    void setRotation(double angle, double roll = 0){
        // roll is executed before the angle
        rollPlane(roll);
        rotatePlane(angle);
    }

    /// debug mode prints the location of the turtle after every move ///
    void setDebug(bool b){
        debug = b;
    }

    /// return the points that were internalized ///
    std::vector<easy3d::Vec<3, double>> getStoredPoints(){
        return storedPoints;
    }

    /// return the stored edges that were internalized ///
    std::vector<std::vector<unsigned int>> getStoredEdges(){
        return storedEdges;
    }

    /// store internalized points to file ///
    void writeToXYZ(const std::string& fileName){
        std::ofstream storageFile;
        storageFile.open(fileName);
        storageFile << "x y z\n";
        for (easy3d::Vec<3, double> p: storedPoints){
            storageFile << p << std::endl;
        }
        storageFile.close();
    }

    /// store internalized points and edges to file ///
    void writeToPly(const std::string& fileName){
        std::ofstream storageFile;
        storageFile.open(fileName);

        // write header
        storageFile << "ply" << std::endl;
        storageFile << "format ascii 1.0" << std::endl;
        storageFile << "element vertex " << storedPoints.size() << std::endl;
        storageFile << "property float x" << std::endl;
        storageFile << "property float y" << std::endl;
        storageFile << "property float z" << std::endl;
        storageFile << "element edge " << storedEdges.size() << std::endl;
        storageFile << "property int vertex1" << std::endl;
        storageFile << "property int vertex2" << std::endl;
        storageFile << "end_header" << std::endl << std::endl;

        // store points
        for (easy3d::Vec<3, double> p: storedPoints){
            storageFile << p << std::endl;
        }

        // store edges
        storageFile << std::endl;
        for (std::vector<unsigned int> i: storedEdges){
            storageFile << i[0] << " " << i[1] << std::endl;
        }
        storageFile.close();
    }

    /// read json file ///
    void readFile(std::string path){
        std::ifstream treeFile(path, std::ifstream::binary);
        nlohmann::json j;
        treeFile >> j;
        treeFile.close();

        setDefaultValues(j["dimensions"]);
        std::string line = translateLine(j["axiom"], j["rules"], j["recursions"]);
        line = cleanLine(line);
        readLine(line);
    }

private:
    // location is a 3d coordinate
    easy3d::Vec<3, double> loc;

    // plane is a 2d plane in a 3d space
    easy3d::Mat<3,3, double> plane;

    // collection of stored points
    std::vector<easy3d::Vec<3, double>> storedPoints;

    // collection of stored edges
    std::vector<std::vector<unsigned int>> storedEdges;

    // default variables
    double fValue = 5;
    double rotateValue = 10;
    double rollValue = 10;

    // prints location at every step
    bool debug = false;

    // commnicate with user
    bool com = true;

    /// step forward ///
    void stepForward(double distance){
//        loc += distance * plane.row(2);
        std::cout << "plane:\n" << plane << std::endl;
        std::cout << "plane * xaxis: " << plane * easy3d::Vec<3, double>(1, 0, 0) << std::endl;
        std::cout << "old location: " << loc << std::endl;
        std::cout << "new location: " << (loc + (plane * easy3d::Vec<3, double>(1, 0, 0) * distance)) << std::endl;

        loc = loc + (plane * easy3d::Vec<3, double>(1, 0, 0) * distance);
    }

    /// rotate ///
    void rotatePlane(double angle){
//        std::cout << "rotating plane with angle " << angle << std::endl;
        angle = angle * M_PI/180;
//        std::cout << "plane: " << plane << std::endl;

        /*easy3d::Vec<3, double> uAxis = plane.row(0);
        easy3d::Vec<3, double> vAxis = plane.row(2);

        easy3d::Vec<3, double> uAxisT = uAxis*cos(angle) + vAxis*sin(angle);
        easy3d::Vec<3, double> vAxisT = -uAxis*sin(angle) + vAxis*cos(angle);

        uAxisT.normalize();
        vAxisT.normalize();

        plane.set_row(0, uAxisT);
        plane.set_row(2, vAxisT);*/

        /// 1: roll to XZ plane
        // project current z-axis onto XY plane;
        easy3d::Vec<3, double> xAxis = plane * easy3d::Vec<3, double>(1, 0, 0);
//        std::cout << "xaxis: " << xAxis << std::endl;
        easy3d::Vec<3, double> xAxis_proj = {xAxis.x, xAxis.y, 0.0};

        // angle of projected x-axis to original x-axis
        double angle_z = (2 * M_PI) - acos(dot(xAxis_proj, easy3d::Vec<3, double>(1,0,0))
                        / (length(xAxis_proj) * length(easy3d::Vec<3, double>(1,0,0))));
        if (isnan(angle_z)){
            angle_z = 0;
        }
//        std::cout << "angle z: " << angle_z << std::endl;
//        std::cout << "angle y: " << angle << std::endl;

        rollPlane(angle_z);

        /// 2: rotation around Y axis
        easy3d::Mat3<double> ry(1);
        ry(0, 0) = std::cos(angle);
        ry(0, 2) = std::sin(angle);
        ry(2, 0) = -std::sin(angle);
        ry(2, 2) = std::cos(angle);

        plane = ry * plane;

        /// 3: roll back from XZ plane
        rollPlane(-angle_z);

//        std::cout << "plane after rotation:\n" << plane << std::endl;
    }

    /// roll ///
    void rollPlane(double rollAngle){
//        std::cout << "rolling plane with angle " << rollAngle << std::endl;
        rollAngle = rollAngle * M_PI/180;
//        std::cout << "plane: " << plane << std::endl;

        easy3d::Mat3<double> rz(1);
        rz(0, 0) = std::cos(rollAngle);
        rz(0, 1) = -std::sin(rollAngle);
        rz(1, 0) = std::sin(rollAngle);
        rz(1, 1) = std::cos(rollAngle);

        plane = rz * plane;

//        std::cout << "plane rolled:\n" << plane << std::endl;
    }

    /// internalize current location ///
    void storeLoc(){
        storedPoints.emplace_back(loc);
    }

    void setDefaultValues(nlohmann::json d){
        if (d.empty()){return;}

        for (const auto& r : d.get<nlohmann::json::object_t>()){
            if (r.first == "forward"){
                fValue = r.second;
            } else if (r.first == "roll"){
                rollValue = r.second;
            } else if (r.first == "rotation"){
                rotateValue = r.second;
            }
        }
    }

    std::string cleanLine(std::string line){
        std::string cleanLine = line;

        // useless increment calls are removed from the loop to avoid bugs and extra computations
        for (int l = line.size(); l > 0; --l) {
            if (line[l] == ']'){
                int nested = 0;

                for (int m = l - 1; m > 0; --m) {
                    if (line[m] == 'F'){break;}
                    else if (line[m] == '[' && nested == 0){cleanLine.erase(m, l - m + 1);}
                    else if (line[m] == ']'){nested ++;}
                    else if (line[m] == '['){nested --;}
                }
            }
        }
        return cleanLine;
    }

    /// translate the complex axiom to a "simple" line ///
    std::string translateLine(nlohmann::json axiom, nlohmann::json rules, nlohmann::json r){

        std::string line = axiom;

        // custom rules need to be applied
        if (rules.empty()){
            std::cout << "WARNING: no rules supplied" << std::endl;
            return line;
        }

        // make a map of the rules
        std::map<std::string, std::string> rulesMap = rules;
        for (const auto& i : rules.get<nlohmann::json::object_t>()){
            rulesMap.insert({i.first, i.second});
        }

        // if the recursion is set in the file to 0 it is changed to 1 to allow one set to be created
        if (r == 0){r = 1;}

        std::string simpleLine = line;

        // TODO allow for multichar rules
        // TODO allow for override rules
        for (int j = 0; j < r; ++j) {
            unsigned int offsetter = 0;
            for(int i = 0; i < line.size(); ++i) {
                std::string s;
                s.push_back(line[i]);

                if (rulesMap.find(s) != rulesMap.end()){

                    std::string replacement = rulesMap[s];

                    simpleLine.insert(i + offsetter, replacement);
                    simpleLine.erase(i + offsetter + replacement.size(), 1);

                    offsetter+= replacement.size() -1;
                }
            }
            line = simpleLine;
        }
        return line;
    }


    /// translate the "simple" line to 3d points ///
    void readLine(std::string line){
        // store starting point
        storeLoc();

        // set trunk location to grow from
        unsigned int trunk = 0;

        // set point counter
        unsigned int pCount = 0;

        // allow to connect recursion to the trunk
        bool returnEdge = false;

        for(int i = 0; i < line.size(); ++i) {
            double override = 0;
            int j = 0;

            // find override values;
            if (line[i + 1] == '('){
                j = 2;
                std::string sValue;
                while (line[i + j] != ')'){
                    sValue += line[i + j];
                    j++;
                }
                override = std::stod(sValue);
            }

            // execute normal stings
            if(line[i] == '['){
                int oNested = 1;
                int cNested = 0;
                int jump = 0;
                for (int k = i; k < line.size(); ++k) {
                    if (line[k] == ']' && oNested == cNested){
                        // recurse turtle
                        Turtle turtle(*this);

                        // allow to debug
                        turtle.setDebug(debug);

                        turtle.readLine(line.substr(i + 1, k - i - 1));

                        // store the points recursion
                        unsigned int offset = storedPoints.size() - 1;
                        for (int l = 1; l < turtle.getStoredPoints().size() ; ++l) {
                            storedPoints.emplace_back(turtle.getStoredPoints()[l]);
                        }

                        // store edges of recursion temporary
                        std::vector<std::vector<unsigned int>> tEdgeList;

                        for (int l = 1; l < turtle.getStoredEdges().size(); ++l) {
                            std::vector<unsigned int> edge = turtle.getStoredEdges()[l];
                            std::vector<unsigned int> nEdge = {edge[0] + offset, edge[1] + offset};
                            tEdgeList.emplace_back(nEdge);
                        }

                        // connect first point of recursion to last point of the trunk
                        std::vector<unsigned int> connector = {trunk, offset + 1};
                        storedEdges.emplace_back(connector);

                        // store temporary edges
                        for (const auto& l: tEdgeList){
                            storedEdges.emplace_back(l);
                        }

                        // set return to true to allow later growth from the trunk
                        returnEdge = true;
                        break;

                    } else if (line[k] == ']'){
                        oNested ++;
                    }else if (line[k] == '['){
                        cNested ++;
                    }
                    jump ++;
                }
                i += jump;

                continue;
            }

            if (line[i] == 'F') {
                // take a step
                if (override == 0){stepForward(fValue);}
                else {stepForward(override);}
                storeLoc();
                trunk = storedPoints.size() - 1;

                if (debug){printLocation();}

                if (returnEdge){
                    // if the point lies after a nesting a correct link has to be set
                    unsigned int p2 = getStoredPoints().size();
                    std::vector<unsigned int> edge = {pCount - 1 , p2 - 1};
                    storedEdges.emplace_back(edge);

                    returnEdge = false;
                } else if (getStoredPoints().size() == 1){
                    // if the first point in a branch no edges are created
                    continue;
                } else {
                    unsigned int p2 = getStoredPoints().size();
                    std::vector<unsigned int> edge = {p2 - 2, p2 - 1};
                    storedEdges.emplace_back(edge);
                }
                pCount = getStoredPoints().size();

            } else if (line[i] == '+') {
                if (override == 0){
                    rotatePlane(rotateValue);
                }
                else {
                    rotatePlane(override);
                }
            } else if (line[i] == '-') {
                if (override == 0){
                    rotatePlane(-rotateValue);
                }
                else {
                    rotatePlane(- override);
                }
            } else if (line[i] == '>') {
                if (override == 0){
                    rollPlane(rollValue);
                }
                else {
                    rollPlane(override);
                }
            } else if (line[i] == '<') {
                if (override == 0){
                    rollPlane(-rollValue);
                }
                else {
                    rollPlane(-override);
                }
            } else if (line[i] == ']'){
                //terminate execution if end of nesting is found
                std::cout << "term" << std::endl;
                break;
            }
            i += j;
        }
    }
};


int main() {
    std::string inputPath = "../test_inputs/in_2.json";
    std::string outputPath = "../export.xyz";
    std::string outputPath2 = "../export2.ply";

    Turtle turtle;
    turtle.setDebug(false);
    turtle.readFile(inputPath);

//    turtle.writeToXYZ(outputPath);
    turtle.writeToPly(outputPath2);
}
