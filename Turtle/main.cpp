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

    void set2Degrees(){
        deg = true;
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

    // set turtle to degrees
    bool deg = false;

    /// step forward ///
    void stepForward(double distance){
        loc = loc + (plane * easy3d::Vec<3, double>(1, 0, 0) * distance);
    }

    /// rotate ///
    void rotatePlane(double angle){
        if (deg){angle = angle * M_PI/180;}

        /// 1: roll to XZ plane
        // project current z-axis onto XY plane;
        easy3d::Vec<3, double> xAxis_orig = {1, 0, 0};
        easy3d::Vec<3, double> xAxis = plane * xAxis_orig;
        easy3d::Vec<3, double> xAxis_proj = {xAxis.x, xAxis.y, 0.0};

        // angle of projected x-axis to original x-axis
//        double angle_z = (2 * M_PI) - acos(dot(xAxis_proj, easy3d::Vec<3, double>(1,0,0))
//                                           / (length(xAxis_proj) * length(easy3d::Vec<3, double>(1,0,0))));
        double angle_z;

        if (xAxis_proj.y < 0){
            angle_z = - acos(dot(xAxis_proj, xAxis_orig) / (length(xAxis_proj) * length(xAxis_orig)));
        }
        else{
            angle_z = acos(dot(xAxis_proj, xAxis_orig) / (length(xAxis_proj) * length(xAxis_orig)));
        }

        if (isnan(angle_z)){
            angle_z = 0;
        }

        if (deg){angle_z = angle_z / (M_PI / 180);}
        rollPlane(-angle_z);

        /// 2: rotation around Y axis
        easy3d::Mat3<double> ry(1);
        ry(0, 0) = std::cos(angle);
        ry(0, 2) = std::sin(angle);
        ry(2, 0) = -std::sin(angle);
        ry(2, 2) = std::cos(angle);

        plane = ry * plane;

        /// 3: roll back from XZ plane
        rollPlane(angle_z);
    }

    /// roll ///
    void rollPlane(double rollAngle){
        if (deg){rollAngle = rollAngle * M_PI/180;}

        easy3d::Mat3<double> rz(1);
        rz(0, 0) = std::cos(rollAngle);
        rz(0, 1) = -std::sin(rollAngle);
        rz(1, 0) = std::sin(rollAngle);
        rz(1, 1) = std::cos(rollAngle);

        plane = rz * plane;
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

                        // using a copy for the edges bypasses potentially rejected edges
                        auto pGraph = graph;

                        // store edges recursion
                        int o = 0;
                        for (const auto& e: turtle.graph.m_edges){
                            unsigned int s;

                            // first edge connects to the trunk
                            if (o == 0){s = trunk;}
                            else {s = e.m_source + offset;}

                            unsigned int t = e.m_target + offset;

                            boost::add_edge(s,t, pGraph);
                            o++;
                        }
                        graph = pGraph;

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
                break;
            }
            i += j;
        }
    }

};


int main() {
    std::string inputPath = "../test_inputs/in_1.json";
    std::string outputPath = "../export.xyz";
    std::string outputPath2 = "../export2.ply";

    // +(279.503)>(56.976)F(0.578)[+(37.797)<(123.617)F(0.577)[-(17.582)>(80.986)F(0.456)[-(3.554)<(12.860)F(0.438)[+(1.248)<(151.516)F(0.622)[+(2.172)>(170.553)F(0.581)[+(19.843)<(179.543)F(0.450)[-(26.309)>(125.900)F(0.569)[+(12.329)>(79.591)F(0.381)[-(15.155)<(63.081)F(0.373)[+(17.183)>(43.179)F(0.441)[+(10.461)<(116.566)F(0.221)[-(3.282)>(99.342)F(0.447)[+(32.211)<(87.235)F(0.348)[+(0.941)>(82.242)F(0.270)[-(319.611)<(46.269)F(0.284)[+(285.687)>(10.595)F(0.155)[-(301.662)>(25.662)F(0.373)[+(331.698)<(15.384)F(0.426)[+(9.012)>(46.667)F(0.301)[-(326.064)<(26.904)F(0.250)[-(14.556)>(35.913)F(0.225)[+(344.192)>(12.850)F(0.295)[-(343.962)<(86.503)F(0.327)[+(19.517)>(71.293)F(0.409)[+(323.485)>(48.759)F(0.193)[-(20.381)>(0.652)F(0.190)[-(324.552)>(1.639)F(0.359)[+(40.021)<(93.436)F(0.431)[+(0.308)>(65.667)F(0.331)[-(47.548)<(8.309)F(0.254)[+(7.296)<(38.360)F(0.297)]]][+(13.562)<(64.052)F(0.480)[-(35.649)<(62.282)F(0.031)]]]]]][+(325.330)<(34.885)F(0.373)]][+(338.692)>(74.184)F(0.394)][+(305.311)>(54.120)F(0.425)]][-(28.890)<(13.180)F(0.401)[-(298.497)>(10.258)F(0.419)]]][+(54.845)>(10.708)F(0.311)[+(287.176)<(65.290)F(0.324)]][+(302.050)>(15.783)F(0.290)[+(11.341)<(64.742)F(0.401)][+(6.224)>(32.641)F(0.364)]]][-(22.244)<(11.844)F(0.261)[+(338.241)<(28.059)F(0.200)[-(334.467)>(35.294)F(0.158)[+(25.734)<(19.319)F(0.389)[-(11.019)>(50.743)F(0.512)[+(305.309)>(20.716)F(0.076)[-(320.080)<(82.902)F(0.379)[+(20.773)>(13.831)F(0.298)[+(319.689)>(30.954)F(0.252)[-(6.941)<(26.627)F(0.305)]]][+(51.455)>(89.454)F(0.207)]][+(9.006)<(20.438)F(0.331)[-(317.176)<(59.169)F(0.306)[+(0.445)>(10.960)F(0.323)]][-(277.844)>(2.202)F(0.315)]]][+(6.667)<(102.114)F(0.362)[+(16.322)<(7.273)F(0.418)[+(2.070)>(103.816)F(0.338)[-(14.196)<(56.163)F(0.145)]]][-(26.022)>(52.020)F(0.321)[-(1.475)<(60.103)F(0.075)]]]]][+(13.001)>(38.722)F(0.436)]]][+(1.726)<(84.826)F(0.371)]]][-(8.978)<(82.582)F(0.380)[-(55.761)<(12.475)F(0.082)[+(29.515)<(24.970)F(0.774)[-(306.118)>(73.494)F(0.269)[+(302.879)<(32.757)F(0.455)[+(31.008)>(2.662)F(0.167)[-(25.911)<(10.366)F(0.359)[-(302.470)<(18.274)F(0.565)][-(6.714)<(105.271)F(0.210)]]][+(6.988)<(120.352)F(0.246)]][+(5.883)<(67.815)F(0.283)]][-(19.581)>(4.407)F(0.575)[+(39.510)>(4.008)F(0.279)[+(5.272)<(33.842)F(0.418)[-(332.304)<(22.772)F(0.223)]][+(10.998)<(16.768)F(0.411)[-(323.720)>(59.397)F(0.164)]]]][-(23.732)<(53.297)F(0.617)]][+(6.540)>(25.373)F(0.931)][+(42.876)>(28.783)F(0.810)[-(323.266)>(48.529)F(0.096)[-(3.223)<(31.738)F(0.238)[+(15.843)>(4.852)F(0.436)[-(8.154)<(23.788)F(0.113)[+(43.412)<(39.589)F(0.442)[+(301.956)<(72.340)F(0.218)]]]][-(1.883)<(73.742)F(0.340)[+(18.614)<(1.499)F(0.394)[+(333.795)>(1.960)F(0.363)][-(12.301)>(63.283)F(0.332)][-(9.045)<(80.066)F(0.071)]][+(355.529)>(78.954)F(0.419)]]][+(338.007)>(33.745)F(0.371)][+(314.115)>(13.112)F(0.415)]][-(15.633)>(117.718)F(0.408)][+(25.039)<(35.370)F(0.457)]]]]]][+(296.978)<(14.535)F(0.357)[+(46.624)>(59.398)F(0.316)][-(35.398)>(187.548)F(0.758)]]][+(28.723)>(60.962)F(0.341)[+(14.409)<(26.757)F(0.518)[-(346.728)>(43.959)F(0.291)[+(50.310)<(79.855)F(0.146)[-(50.688)>(15.125)F(0.169)[+(26.845)>(90.138)F(0.364)[-(10.652)<(35.211)F(0.386)[-(9.766)>(44.535)F(0.256)[+(51.504)>(17.659)F(0.215)[-(63.551)<(52.767)F(0.361)[+(30.103)<(88.973)F(0.138)]]]][+(29.526)<(103.532)F(0.267)]]][+(9.234)<(25.097)F(0.085)[+(9.195)>(84.290)F(0.423)]]]][-(0.837)>(24.572)F(0.359)[+(3.292)<(29.764)F(0.372)][+(20.637)<(33.954)F(0.361)]]]]][-(304.179)<(0.831)F(0.368)[+(1.139)<(55.172)F(0.197)[-(0.364)>(87.525)F(0.297)[+(348.838)<(62.405)F(0.167)]][-(7.600)<(33.690)F(0.531)]]]]][-(2.733)<(93.002)F(0.528)[-(339.568)>(1.348)F(0.427)[+(316.788)>(14.677)F(0.237)[+(12.430)<(71.049)F(0.534)[-(55.559)>(33.166)F(0.986)[+(63.998)>(192.608)F(0.076)]]][-(307.698)<(24.411)F(0.337)]][+(22.177)<(80.953)F(0.499)]][-(301.798)>(13.664)F(0.451)][-(21.520)<(70.655)F(0.429)[+(27.596)>(26.128)F(0.676)[-(68.419)>(54.055)F(0.938)[+(45.647)>(24.538)F(0.065)]][-(5.712)>(280.781)F(0.191)]][+(31.297)>(40.731)F(0.691)]]]]][+(20.300)>(55.804)F(0.446)[+(14.458)<(42.630)F(0.561)[+(4.317)<(52.154)F(0.201)[-(330.902)>(6.279)F(0.356)[+(331.869)>(61.803)F(0.498)[-(336.535)<(53.798)F(0.266)[-(0.764)>(51.211)F(0.308)[+(4.830)<(58.308)F(0.283)[-(17.672)>(52.179)F(0.390)[-(3.511)>(39.716)F(0.385)[+(15.594)<(7.014)F(1.355)]][+(25.325)<(54.524)F(0.201)]]][+(329.188)>(3.892)F(0.405)[-(10.981)>(33.379)F(0.242)[-(315.920)>(21.636)F(0.166)]]]]][-(32.399)>(31.375)F(0.092)[-(303.978)<(37.646)F(0.309)]][-(333.790)<(65.417)F(0.269)]]]][-(311.932)>(62.904)F(0.394)[+(312.244)<(37.325)F(0.213)[-(12.784)>(61.867)F(0.258)[-(326.312)>(25.583)F(0.607)]][+(4.922)<(38.736)F(0.364)]]]]]]]][+(21.109)<(32.221)F(0.448)[-(5.080)<(114.621)F(0.664)]]][+(28.284)<(45.380)F(0.352)[-(319.779)>(82.475)F(0.340)[+(331.125)<(51.671)F(0.439)]]]]][-(12.204)>(278.499)F(0.393)[-(9.420)<(188.421)F(0.430)[+(1.283)>(89.238)F(0.421)[+(19.061)>(103.091)F(0.420)[-(12.248)<(179.899)F(0.399)[-(303.667)<(75.984)F(0.308)[+(299.419)<(32.727)F(0.538)[+(16.494)>(50.937)F(0.770)[-(317.456)>(69.043)F(0.332)[+(343.247)<(4.613)F(0.408)[-(339.881)>(17.353)F(0.269)[+(3.058)<(15.676)F(0.283)[+(281.555)<(62.043)F(0.886)]][+(20.178)>(69.005)F(0.136)]]][+(356.558)<(42.478)F(0.479)]][+(40.149)<(19.507)F(0.403)[-(23.484)>(295.583)F(0.357)[-(28.217)<(261.743)F(0.417)[-(14.381)>(61.473)F(0.132)]][-(1.591)<(62.159)F(0.382)][-(10.064)<(324.179)F(0.457)[-(22.612)>(266.205)F(0.118)]][-(306.675)<(313.716)F(0.424)]][-(26.720)>(37.927)F(0.078)]][+(9.121)>(32.162)F(0.226)[-(17.424)>(64.204)F(0.386)[+(11.377)<(135.833)F(0.412)[+(26.834)>(13.159)F(0.319)[-(318.230)>(27.708)F(0.113)]]]]]][-(289.445)>(294.670)F(0.374)[+(309.342)>(49.806)F(0.391)]][+(10.851)>(291.796)F(0.678)][+(19.665)>(159.641)F(0.461)[-(26.405)<(139.759)F(0.430)]]]][-(32.321)<(118.999)F(0.820)[+(11.126)>(275.694)F(0.156)[+(46.316)<(278.445)F(0.264)]]]][-(301.509)>(52.286)F(0.332)[+(338.250)<(56.403)F(0.314)[-(34.843)<(5.089)F(0.350)[+(15.307)>(45.224)F(0.360)[-(30.785)<(274.684)F(0.528)[+(14.897)>(90.234)F(0.155)][-(298.833)>(17.325)F(0.370)]]][-(20.265)<(226.237)F(0.665)]]][+(323.541)<(342.764)F(0.209)[-(333.916)>(318.482)F(0.303)]]][+(22.956)<(92.394)F(0.374)[-(46.016)>(69.318)F(0.448)[+(14.758)<(272.289)F(0.682)[+(31.134)>(263.297)F(0.184)[-(0.023)<(268.619)F(0.202)]]][+(22.931)<(54.442)F(0.251)[+(24.030)>(15.421)F(0.188)]]][+(16.182)>(26.765)F(0.253)[-(314.634)>(39.910)F(0.214)]]]]]][+(9.388)<(272.345)F(0.622)]]]]]][+(16.770)>(160.815)F(0.425)[-(33.912)<(93.095)F(0.665)][-(33.912)<(93.095)F(0.665)]]]]
    // +(324.736)>(45.000)F(1.732)[+(35.264)>(45.000)F(1.000)][F(1.732)[+(35.264)F(1.414)][-(9.736)<(45.000)F(1.414)[>(90.000)F(1.414)[-(45.000)<(90.000)F(1.000)]][-(45.000)F(1.000)[F(1.000)[F(1.000)]]]]]

    Turtle turtle;
    turtle.setDebug(false);
    turtle.set2Degrees();
    turtle.readFile(inputPath);
//    turtle.writeToXYZ(outputPath);
    turtle.writeToPly(outputPath2);
}
