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
        rollValue = other.rotateValue;

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
    void setDebug(){
        debug = true;
    }

    /// return the points that were internalized ///
    std::vector<easy3d::Vec<3, double>> getStoredPoints(){
        return storedPoints;
    }

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

    void writeToPly(const std::string& fileName){
        std::ofstream storageFile;
        storageFile.open(fileName);
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

        for (easy3d::Vec<3, double> p: storedPoints){
            storageFile << p << std::endl;
        }

        storageFile << std::endl;
        for (std::vector<unsigned int> i: storedEdges){
            storageFile << i[0] << " " << i[1] << std::endl;
        }
        storageFile.close();
    }

    /// read json file ///
    void readFile(nlohmann::json j){
        setDefaultValues(j["dimensions"]);
        std::string line = translateLine(j["axiom"], j["rules"], j["recursions"]);

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

    //debug prints location at every step
    bool debug = false;

    /// step forward ///
    void stepForward(double distance){
        loc += distance * plane.row(2);
    }

    /// rotate ///
    void rotatePlane(double angle){
        angle = angle * M_PI/180;

        easy3d::Vec<3, double> uAxis = plane.row(0);
        easy3d::Vec<3, double> vAxis = plane.row(2);

        easy3d::Vec<3, double> uAxisT = uAxis*cos(angle) - vAxis*sin(angle);
        easy3d::Vec<3, double> vAxisT = uAxis*sin(angle) + vAxis*cos(angle);

        uAxisT.normalize();
        vAxisT.normalize();

        plane.set_row(0, uAxisT);
        plane.set_row(2, vAxisT);
    }

    /// roll ///
    void rollPlane(double rollAngle){
        rollAngle = rollAngle * M_PI/180;

        easy3d::Vec<3, double> uAxis = plane.row(0);
        easy3d::Vec<3, double> wAxis = plane.row(1);

        easy3d::Vec<3, double> uAxisT = uAxis*cos(rollAngle) - wAxis*sin(rollAngle);
        easy3d::Vec<3, double> wAxisT = uAxis*sin(rollAngle) + wAxis*cos(rollAngle);

        uAxisT.normalize();
        wAxisT.normalize();

        plane.set_row(0, uAxisT);
        plane.set_row(1, wAxisT);
    }

    /// internalize current location ///
    void storeLoc(){
        storedPoints.emplace_back(loc);
    }

    void storeEdge(){

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

    /// translate the complex axiom to a "simple" line ///
    std::string translateLine(nlohmann::json axiom, nlohmann::json rules, nlohmann::json r){
        std::string basicChars = "F+-<>()[]1234567890";
        std::string line = axiom;

        // custom rules need to be applied
        if (rules.empty()){
            std::cout << "WARNING: custom rules are present in axiom but no explanation has been supplied" << std::endl;
            return line;
        }

        std::map<std::string, std::string> rulesMap = rules;
        for (const auto& i : rules.get<nlohmann::json::object_t>()){
            rulesMap.insert({i.first, i.second});
        }

        if (r == 0){r = 1;}

        std::string simpleLine = line;

        for (int j = 0; j < r; ++j) {
            int offsetter = 0;
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

        unsigned int basepoint = 0;
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
                        Turtle turtle(*this);

                        if (debug){turtle.setDebug();}

                        turtle.readLine(line.substr(i + 1, k - i - 1));

                        unsigned int connection = storedPoints.size() - 1;
                        for (int l = 1; l < turtle.getStoredPoints().size() ; ++l) {
                            storedPoints.emplace_back(turtle.getStoredPoints()[l]);
                        }

                        std::vector<std::vector<unsigned int>> tEdgeList;

                        for (int l = 1; l < turtle.getStoredEdges().size(); ++l) {
                            std::vector<unsigned int> edge = turtle.getStoredEdges()[l];
                            std::vector<unsigned int> nEdge = {edge[0] + connection, edge[1] + connection};
                            tEdgeList.emplace_back(nEdge);
                        }

                        std::vector<unsigned int> connector = {connection, connection + 1};
                        storedEdges.emplace_back(connector);

                        for (const auto& l: tEdgeList){
                            storedEdges.emplace_back(l);
                        }


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

                if (debug){printLocation();}


                if (returnEdge){
                    // if the point lies after a nesting a correct link has to be set
                    unsigned int connection = getStoredPoints().size();
                    std::vector<unsigned int> edge = {basepoint - 1 , connection - 1};
                    storedEdges.emplace_back(edge);

                    returnEdge = false;
                } else if (getStoredPoints().size() == 1){
                    // if the first point in a branch no edges are created
                    continue;
                } else {
                    unsigned int connection = getStoredPoints().size();
                    std::vector<unsigned int> edge = {connection - 2, connection - 1};
                    storedEdges.emplace_back(edge);
                }
                basepoint = getStoredPoints().size();

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
    std::string inputPath = "../test_inputs/in_1.json";
    std::string outputPath = "../export.xyz";
    std::string outputPath2 = "../export.ply";


    std::ifstream treeFile(inputPath, std::ifstream::binary);
    nlohmann::json j;
    treeFile >> j;
    treeFile.close();

    Turtle turtle;
    //turtle.setDebug();
    turtle.readFile(j);

    turtle.writeToXYZ(outputPath);
    turtle.writeToPly(outputPath2);
}
