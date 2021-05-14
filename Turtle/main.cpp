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

struct SGraphVertexProp
{
    easy3d::Vec<3, double>  cVert;
    std::size_t nParent;
    double lengthOfSubtree;

    double radius; // used only by the smoothed skeleton
    bool   visited;
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

    /// construct a memoryless turtle based on another turtle ///
    Turtle (Turtle const &other){
        *this = other;
        graph.clear();
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
    auto getStoredPoints(){
        return graph.m_vertices;
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

        for (auto & m_vertice : graph.m_vertices) {
            storageFile << m_vertice.m_property.cVert << std::endl;
        }
        storageFile.close();
    }

    /// store internalized points and edges to file ///
    void writeToPly(const std::string& fileName){
        std::ofstream storageFile;
        storageFile.open(fileName);

        auto verts = graph.m_vertices;
        auto edges = graph.m_edges;

        // write header
        storageFile << "ply" << std::endl;
        storageFile << "format ascii 1.0" << std::endl;
        storageFile << "element vertex " << verts.size() << std::endl;
        storageFile << "property float x" << std::endl;
        storageFile << "property float y" << std::endl;
        storageFile << "property float z" << std::endl;
        storageFile << "element edge " << edges.size() << std::endl;
        storageFile << "property int vertex1" << std::endl;
        storageFile << "property int vertex2" << std::endl;
        storageFile << "end_header" << std::endl << std::endl;

        // store points
        for (auto & vertex : verts) {
            storageFile << vertex.m_property.cVert << std::endl;
        }

        storageFile << std::endl;
        for (auto edge: edges){
            storageFile << edge.m_source << " " << edge.m_target << std::endl;
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
    void storeLoc(unsigned int parent = -1){
        SGraphVertexProp p1;
        p1.cVert = loc;
        if (parent >= 0){p1.nParent = parent;}
        else {p1.nParent = NULL;}

        graph.m_vertices.emplace_back(p1);
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
        // TODO improve [[]] cases

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
        std::cout << line << std::endl;

        // store starting point
        storeLoc(0);

        // set trunk location to grow from
        unsigned int trunk = 0;

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
                        auto cVertexList = turtle.graph.m_vertices;

                        // store the points recursion
                        unsigned int offset = graph.m_vertices.size() - 1;

                        // ignore point 0 (is the same point as branch point)
                        for (int l = 1; l < turtle.graph.m_vertices.size(); ++l) {
                            if (l == 1){cVertexList[1].m_property.nParent = trunk;}
                            else {cVertexList[l].m_property.nParent += offset;}
                            graph.m_vertices.emplace_back(cVertexList[l]);
                        }

                        auto cEdgeList = turtle.graph.m_edges;

                        // store edges recursion
                        int o = 0;
                        for (const auto& e: cEdgeList){
                            unsigned int s;

                            // first edge connects to the trunk
                            if (o == 0){s = trunk;}
                            else {s = e.m_source + offset;}

                            unsigned int t = e.m_target + offset;
                            boost::add_edge(s,t, graph);
                            o++;
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

                // if this is not the final step of a straight line piece no point nor edge is created
                if (line[i+1] == 'F'){continue;}

                auto vertList = graph.m_vertices;
                int n_self = vertList.size();

                if (!returnEdge){ // if not the start of a branch
                    // the parent is always the index number of the one in front
                    int n_parent = n_self - 1;

                    storeLoc(n_parent);
                    boost::add_edge(n_parent, n_self, graph);
                } else { // if start of a branch
                    // the parent is always the trunk point
                    returnEdge = false;

                    storeLoc(trunk);
                    boost::add_edge(trunk, n_self, graph);
                }
                trunk = graph.m_vertices.size() - 1;

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

    Turtle turtle;
    turtle.setDebug(false);
    turtle.readFile(inputPath);
    turtle.writeToXYZ(outputPath);
    turtle.writeToPly(outputPath2);
}
