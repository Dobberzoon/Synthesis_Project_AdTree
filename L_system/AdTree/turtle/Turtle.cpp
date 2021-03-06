//
// Created by jasper van der vaart on 18/05/2021.
//

#include "Turtle.h"

Turtle::Turtle(float x, float y, float z) {
    {
        loc = {x, y, z};
        plane.set_row(0, easy3d::vec3{1, 0, 0});
        plane.set_row(1, easy3d::vec3{0, 1, 0});
        plane.set_row(2, easy3d::vec3{0, 0, 1});
    }
}

Turtle::Turtle(easy3d::vec3 p) {
    loc = p;
    plane.set_row(0, easy3d::vec3{1, 0, 0});
    plane.set_row(1, easy3d::vec3{0, 1, 0});
    plane.set_row(2, easy3d::vec3{0, 0, 1});
}

Turtle::Turtle(Turtle const &other) {
    *this = other;
    graph.clear();
}

void Turtle::printLocation() const {
    std::cout << "current location of turtle is: (";
    std::cout << loc.x << ", " << loc.y << ", " << loc.z << ")" << std::endl;
}

void Turtle::setStartingValues(const nlohmann::json &j) {

    // set anchorpoint
    nlohmann::json p = j["anchor"];

    if (p.size() < 2) {
        std::cout << "no anchor is supplied in json, 0,0,0 is used as anchor!" << std::endl;
    } else if (p.size() == 2) {
        std::cout << "2d coordinates are supplied as anchor, coordinates are used as x,y!" << std::endl;
        loc = {p[0], p[1], 0};
        anchor = {p[0], p[1], 0};
    } else if (p.size() == 3) {
        loc = {p[0], p[1], p[2]};
        anchor = {p[0], p[1], p[2]};
    }

    nlohmann::json r = j["radius"];

    if (r.size() == 0) {
        std::cout << "no radius is supplied in json" << std::endl;
        trunkR = 0;
    } else {
        trunkR = j["radius"];
    }
}

void Turtle::setRotation(float angle, float roll) {
    // roll is executed before the angle
    rollPlane(roll);
    rotatePlane(angle);
}

void Turtle::setDebug(bool b) {
    debug = b;
}

void Turtle::set2Degrees() {
    deg = true;
}

std::vector<easy3d::vec3> Turtle::getStoredPoints() {
    std::vector<easy3d::vec3> pointList;
    for (const auto &p: graph.m_vertices) { pointList.emplace_back(p.m_property.cVert); }
    return pointList;
}

std::vector<std::vector<unsigned int>> Turtle::getStoredEdges() {
    return storedEdges;
}

Graph Turtle::getGraph() const {
    return graph;
}

easy3d::vec3 Turtle::getAnchor() const {
    return anchor;
}

float Turtle::getRadius() const {
    return trunkR;
}

float Turtle::getHeight() const{
    return TreeHeight;
}

float Turtle::getBoundingDistance() const{
    return BoundingDistance;
}

void Turtle::writeToXYZ(const std::string &fileName) {
    std::ofstream storageFile;
    storageFile.open(fileName);
    storageFile << "x y z\n";

    for (auto &m_vertice : graph.m_vertices) {
        storageFile << m_vertice.m_property.cVert << std::endl;
    }
    storageFile.close();
}


void Turtle::writeToPly(const std::string &fileName) {
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
    for (auto &vertex : verts) {
        storageFile << vertex.m_property.cVert << std::endl;
    }

    storageFile << std::endl;
    for (const auto &edge: edges) {
        storageFile << edge.m_source << " " << edge.m_target << std::endl;
    }

    storageFile.close();
}


void Turtle::readFile(const std::string &path) {
    std::ifstream treeFile(path, std::ifstream::binary);
    nlohmann::json j;
    treeFile >> j;
    treeFile.close();

    if  (j["degrees"].empty()){std::cout << "no degree indidcator has been supplied" << std::endl;}
    else {deg = j["degrees"];}

    if (j["trunk"].empty()){std::cout << "No trunk data has been supplied" << std::endl;}
    else{setStartingValues(j["trunk"]);}

    if (j["dimensions"].empty()){std::cout << "No default dimensions data has been supplied" << std::endl;}
    else{setDefaultValues(j["dimensions"]);}

    std::string line = translateLine(j["axiom"], j["rules"], j["recursions"]);
    line = cleanLine(line);
    readLine(line);
    updateStats();
}

void Turtle::stepForward(float distance) {
    loc = loc + (plane * easy3d::vec3(1, 0, 0) * distance);
}

/// rotate ///
void Turtle::rotatePlane(float angle) {
    if (deg) { angle =  angle * (float) M_PI / 180; }

    /// 1: roll to XZ plane
    // project current z-axis onto XY plane;
    easy3d::vec3 xAxis_orig = {1, 0, 0};
    easy3d::vec3 xAxis = plane * xAxis_orig;
    easy3d::vec3 xAxis_proj = {xAxis.x, xAxis.y, 0.0};

    // angle of projected x-axis to original x-axis
    float angle_z;

    if (xAxis_proj.y < 0) {
        angle_z = -acos(dot(xAxis_proj, xAxis_orig) / (length(xAxis_proj) * length(xAxis_orig)));
    } else {
        angle_z = acos(dot(xAxis_proj, xAxis_orig) / (length(xAxis_proj) * length(xAxis_orig)));
    }
    // angle_z is close to 0
    // problem: nan
    if (std::isnan(angle_z)) {
        angle_z = 0;
    }
    // vector points (almost) straight up/down
    // problem: incorrect angle_z in 2d cases
    if (abs(xAxis.z) - 1 < 0.1 && abs(xAxis.z) - 1 > - 0.1 &&
        abs(xAxis.x)  == 0 ||
        abs(xAxis.z) - 1 < 0.1 && abs(xAxis.z) - 1 > - 0.1 &&
        abs(xAxis.y) == 0) {
        angle_z = 0;
    }

    if (deg) { angle_z = angle_z / (float) (M_PI / 180); }
    rollPlane(-angle_z);

    /// 2: rotation around Y axis
    easy3d::Mat3<float> ry(1);
    ry(0, 0) = std::cos(angle);
    ry(0, 2) = std::sin(angle);
    ry(2, 0) = -std::sin(angle);
    ry(2, 2) = std::cos(angle);

    plane = ry * plane;

    /// 3: roll back from XZ plane
    rollPlane(angle_z);
}

void Turtle::rollPlane(float rollAngle) {
    if (deg) { rollAngle = rollAngle * (float) M_PI / 180; }

    easy3d::Mat3<float> rz(1);
    rz(0, 0) = std::cos(rollAngle);
    rz(0, 1) = -std::sin(rollAngle);
    rz(1, 0) = std::sin(rollAngle);
    rz(1, 1) = std::cos(rollAngle);

    plane = rz * plane;
}

void Turtle::updateStats(){
    // update graph statistics
    for (auto p: graph.m_vertices){
        easy3d::vec3 point = p.m_property.cVert;

        if ((point.z - anchor.z) > TreeHeight)
            TreeHeight = point.z - anchor.z;
        if (std::sqrt(point.distance2(anchor)) > BoundingDistance)
            BoundingDistance = std::sqrt(point.distance2(anchor));
    }
}

void Turtle::storeLoc(unsigned int parent) {
    // add vert to graph
    SGraphVertexProp p1;
    p1.cVert = loc;
    if (parent >= 0) { p1.nParent = parent; }
    else { p1.nParent = NULL; }

    graph.m_vertices.emplace_back(p1);


}

void Turtle::setDefaultValues(const nlohmann::json &d) {
    if (d.empty()) { return; }

    for (const auto &r : d.get<nlohmann::json::object_t>()) {
        if (r.first == "forward") {
            fValue = r.second;
        } else if (r.first == "roll") {
            rollValue = r.second;
        } else if (r.first == "rotation") {
            rotateValue = r.second;
        }
    }
}

std::string Turtle::cleanLine(std::string line) {
    std::string cleanLine = line;

    // useless increment calls are removed from the loop to avoid bugs and extra computations
    for (int l = line.size(); l > 0; --l) {
        if (line[l] == ']') {
            int nested = 0;

            for (int m = l - 1; m > 0; --m) {
                if (line[m] == 'F') { break; }
                else if (line[m] == '[' && nested == 0) { cleanLine.erase(m, l - m + 1); }
                else if (line[m] == ']') { nested++; }
                else if (line[m] == '[') { nested--; }
            }
        }
    }
    return cleanLine;
}


std::string Turtle::translateLine(const nlohmann::json &axiom, const nlohmann::json &rules, nlohmann::json r) {

    std::string line = axiom;

    // custom rules need to be applied
    if (rules.empty()) {
        std::cout << "WARNING: no rules supplied" << std::endl;
        return line;
    }

    // make a map of the rules
    std::map<std::string, std::string> rulesMap = rules;
    for (const auto &i : rules.get<nlohmann::json::object_t>()) {
        rulesMap.insert({i.first, i.second});
    }

    // if the recursion is set in the file to 0 it is changed to 1 to allow one set to be created
    if (r == 0) { r = 1; }

    std::string simpleLine = line;

    // TODO allow for multichar rules
    // TODO allow for override rules

    for (int j = 0; j < r; ++j) {
        unsigned int offsetter = 0;
        for (unsigned int i = 0; i < line.size(); ++i) {
            std::string s;
            s.push_back(line[i]);

            if (rulesMap.find(s) != rulesMap.end()) {

                std::string replacement = rulesMap[s];

                simpleLine.insert(i + offsetter, replacement);
                simpleLine.erase(i + offsetter + replacement.size(), 1);

                offsetter += replacement.size() - 1;
            }
        }
        line = simpleLine;
    }
    return line;
}


void Turtle::readLine(std::string line) {
    // store starting point
    storeLoc(0);

    // set trunk location to grow from
    unsigned int trunk = 0;

    // allow to connect recursion to the trunk
    bool returnEdge = false;

    for (unsigned int i = 0; i < line.size(); ++i) {
        float override = 0;
        int j = 0;

        // find override values;
        if (line[i + 1] == '(') {
            j = 2;
            std::string sValue;
            while (line[i + j] != ')') {
                sValue += line[i + j];
                j++;
            }
            override = std::stof(sValue);
        }

        // execute normal stings
        if (line[i] == '[') {
            int oNested = 1;
            int cNested = 0;
            int jump = 0;

            for (unsigned int k = i; k < line.size(); ++k) {
                if (line[k] == ']' && oNested == cNested) {
                    // recurse turtle
                    Turtle turtle(*this);

                    // allow to debug
                    turtle.setDebug(debug);

                    turtle.readLine(line.substr(i + 1, k - i - 1));
                    auto cVertexList = turtle.graph.m_vertices;

                    // store the points recursion
                    unsigned int offset = graph.m_vertices.size() - 1;

                    // ignore point 0 (is the same point as branch point)
                    for (unsigned int l = 1; l < turtle.graph.m_vertices.size(); ++l) {
                        if (l == 1) { cVertexList[1].m_property.nParent = trunk; }
                        else { cVertexList[l].m_property.nParent += offset; }
                        graph.m_vertices.emplace_back(cVertexList[l]);
                    }

                    // using a copy for the edges bypasses potentially rejected edges
                    auto pGraph = graph;

                    // store edges recursion
                    int o = 0;
                    for (const auto &e: turtle.graph.m_edges) {
                        unsigned int s;

                        // first edge connects to the trunk
                        if (o == 0) { s = trunk; }
                        else { s = e.m_source + offset; }

                        unsigned int t = e.m_target + offset;

                        boost::add_edge(s, t, pGraph);
                        o++;
                    }
                    graph = pGraph;

                    // set return to true to allow later growth from the trunk
                    returnEdge = true;
                    break;

                } else if (line[k] == ']') {
                    oNested++;
                } else if (line[k] == '[') {
                    cNested++;
                }
                jump++;
            }
            i += jump;

            continue;
        }

        if (line[i] == 'F') {
            // take a step
            if (override == 0) { stepForward(fValue); }
            else { stepForward(override); }

            // if this is not the final step of a straight line piece no point nor edge is created
            if (line[i + 1] == 'F') { continue; }

            auto vertList = graph.m_vertices;
            int n_self = vertList.size();

            if (!returnEdge) { // if not the start of a branch
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
            if (override == 0) {
                rotatePlane(rotateValue);
            } else {
                rotatePlane(override);
            }
        } else if (line[i] == '-') {
            if (override == 0) {
                rotatePlane(-rotateValue);
            } else {
                rotatePlane(-override);
            }
        } else if (line[i] == '>') {
            if (override == 0) {
                rollPlane(rollValue);
            } else {
                rollPlane(override);
            }
        } else if (line[i] == '<') {
            if (override == 0) {
                rollPlane(-rollValue);
            } else {
                rollPlane(-override);
            }
        } else if (line[i] == ']') {
            //terminate execution if end of nesting is found
            break;
        }
        i += j;
    }
}
