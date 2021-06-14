//
// Created by:
// noortje van der Horst
// Jasper van der Vaart
// Haoyang Dong
//

#include "L-system.h"



using namespace boost;
using namespace easy3d;


Lsystem::Lsystem()
    {
        Lstring_ = "";
        axiom = "";
        rules = {};
        degrees_ = false;
    }


void Lsystem::printLsystem() {
    std::cout << "printing L-system..." << std::endl;
    std::cout << "string: " << Lstring_ << std::endl;
    std::cout << "axiom:  " << axiom << std::endl;
    std::cout << "rules:  " << std::endl;
    for (auto rule:rules){
        std::cout << "\t" << rule.first << ": " << rule.second << std::endl;
    }
    std::cout << "printing L-system: done" << std::endl;
}


void Lsystem::lsysToJson(const std::string &filename) {
    std::cout << "L-system: writing to file..." << std::endl;

    nlohmann::json j;

    j["recursions"] = rec_;
    j["axiom"] = axiom;        // Lstring_
    j["rules"] = rules;        // empty for now

    j["trunk"] = {{"anchor", {anchor_.x, anchor_.y, anchor_.z}},
                  {"radius", radius_}};

    j["dimensions"] = { {"forward", forward_},
                        {"rotation", rotation_},
                        {"roll", roll_} };

    j["degrees"] = degrees_;

//    std::cout << std::setw(4) << j << std::endl;
//    std::cout << "dir: " << output_dir + filename << std::endl;

    std::ofstream storageFile(filename);
    storageFile << std::setw(4) << j << std::endl;
    storageFile.close();

    std::cout << "writing to file: done" << std::endl;
}


void Lsystem::lsysToText(const std::string &filename){};


void Lsystem::readSkeleton(Skeleton *skel, bool deg, bool grow) {
    std::cout << "\n---------- initializing L-system ----------" << std::endl;
    std::cout << "nr. vertices of simplified skeleton: " << num_vertices(skel->get_simplified_skeleton()) << std::endl;

    // determine if growing
    grow_ = grow;

    /// set parameters
    degrees_ = deg;
    if (grow_){
        radius_ = skel->getRadius()*(1 + grow_sp * grow_co);
    } else {
        radius_ = skel->getRadius();
    }
    anchor_ = skel->getAnchor();

    if (grow_) {
        // l-branches
        buildBranches(skel);

        // rules
        buildRules(skel, 3);
    }

    /// convert skeleton to Lsystem
    graph_lsys = skel->get_simplified_skeleton();
    root_ = skel->get_root();

    SGraphVertexDescriptor root = skel->get_root();
    vec3 coords_root = skel->get_simplified_skeleton()[root].cVert;
    traverse(root, root, skel);
    axiom = Lstring_;  // initially axiom is the full string

    std::cout << "converting to L-system: done" << std::endl;
}


SGraphVertexDescriptor Lsystem::traverse(SGraphVertexDescriptor prevV,
                       SGraphVertexDescriptor startV,
                       Skeleton *skel){
    // write movement from prevV to nextV to Lstring
    // will not write when prevV is a leaf or the root (preventing doubles and the root writing to itself)
    if (grow_){
        if (sprout(sprout_pos, prevV, skel)) {
            Lstring_ += selectRule(prevV, startV, skel);
        }
    }
    /// write movement towards current node, before traversing further
    writeMovement(prevV, startV, skel, 3);
    std::cout << "orig node: " << startV << std::endl;

    vec3 start_coords = skel->get_simplified_skeleton()[startV].cVert;
    std::vector<SGraphVertexDescriptor> children;

    // skip if node is leaf
    if ((out_degree(startV, skel->get_simplified_skeleton()) == 1)
        && (startV != skel->get_simplified_skeleton()[startV].nParent)) {
        return startV;
    }
    else {
        /// find all next nodes
        std::pair<Graph::out_edge_iterator, Graph::out_edge_iterator> outei =
                boost::out_edges(startV, skel->get_simplified_skeleton());
        for (auto eit = outei.first; eit!=outei.second; ++eit){
            if (boost::target(*eit, skel->get_simplified_skeleton())!=
                skel->get_simplified_skeleton()[startV].nParent) {
                children.push_back(boost::target(*eit, skel->get_simplified_skeleton()));
            }
        }

        std::cout << "orig nexts: " ;
        for (auto nnd:children){
            std::cout << nnd << " ";
        }
        std::cout << "\n" ;

        /// start node has one child: straight segment
        if (out_degree(startV, skel->get_simplified_skeleton()) == 1) {
            return traverse(startV, children[0], skel);
        }
        /// start node has multiple children: beginning of 2 or more branches
        else {
            SGraphVertexDescriptor leaf;
            // also write all the other children
            for (int nChild = 0; nChild < children.size(); ++nChild) {
                Lstring_ += "[";
                graph_lsys[children[nChild]].lstring["nesting"] += "[";
                leaf = traverse(startV, children[nChild], skel);
                graph_lsys[leaf].lstring["nesting"] += "]";
                Lstring_ += "]";
            }
            return leaf;
        }
    }
}


std::tuple<double, double, double> Lsystem::moveToNext(SGraphVertexDescriptor startV,
                                                       SGraphVertexDescriptor nextV,
                                                       Skeleton *skel) {
    // end result: {angle around y-axis, angle anround z-axis, distance}
    std::tuple<double, double, double> movement{0, 0, 0};

    // get previous node (parent of start)
    SGraphVertexDescriptor prevV = skel->get_simplified_skeleton()[startV].nParent;

    /// get relative movement from start to next node
    // exclude parent as second node (next)
    // because is it's own parent, so vector between start & next would be 0
    if (nextV != skel->get_simplified_skeleton()[nextV].nParent) {
        // get coordinates of the two nodes
        vec3 coords_start = skel->get_simplified_skeleton()[startV].cVert;
        vec3 coords_next = skel->get_simplified_skeleton()[nextV].cVert;
        vec3 coords_prev = skel->get_simplified_skeleton()[prevV].cVert;

        // get distance between the two nodes
        float branch_length = easy3d::distance(coords_start, coords_next);

        /// compute vectors previous <--> start and start <--> next
        // compute the vector (difference) between the two nodes (start & next)
        vec3 to_target = (coords_next - coords_start);
        // compute the vector of the previous edge
        vec3 to_origin = (coords_start - coords_prev);

        vec3 xaxis = {1, 0, 0};
        vec3 yaxis = {0, 1, 0};
        vec3 zaxis = {0, 0, 1};

        /// get rotation around Z
        // project to xy plane
        vec3 to_origin_xy = {to_origin.x, to_origin.y, 0};
        vec3 to_target_xy = {to_target.x, to_target.y, 0};

        // find angle between planar vectors & x-axis, around the z-axis (radians)
        double angle_z_orig = getZAngle(to_origin_xy);
        double angle_z_target = getZAngle(to_target_xy);

        // todo: make 360 --> 0?
        // todo: 360+ or -360-?

        /// get rotation around Y
        // rotate vectors to XZ plane
        vec3 to_origin_xz = easy3d::mat3::rotation(zaxis, -angle_z_orig) * to_origin;
        vec3 to_target_xz = easy3d::mat3::rotation(zaxis, -angle_z_target) * to_target;

        // find angle between planar vectors & x-axis, around the y-axis
        double angle_y_orig = getYAngle(to_origin_xz);
        double angle_y_target = getYAngle(to_target_xz);

        /// get relative rotations
        double angle_diff_z = angle_z_target - angle_z_orig;
        double angle_diff_y = angle_y_target - angle_y_orig;

        // may still need these later, switched off for now
        bool debug_print = false;
        if (debug_print) {

            std::cout << "\n---------- computing translation ----------" << std::endl;
            std::cout << "start node: (" << startV << ") " << coords_start
                      << " --> next: (" << nextV << ") " << coords_next
                      << " | previous: (" << prevV << ") " << coords_prev << std::endl;

            std::cout << "\nto origin proj XZ (y): " << to_origin_xz << std::endl;
            std::cout << "to target proj XZ (y): " << to_target_xz << std::endl;

            std::cout << "to_origin: " << to_origin << " , length: " << length(to_origin) << std::endl;
            std::cout << "to_target: " << to_target << " , length: " << length(to_target) << std::endl;
            if (length(to_origin) == 0) {
                to_origin = {1, 0, 0};
            }
            vec3 next_vec = easy3d::mat3::rotation(0, angle_diff_y, angle_z_orig + angle_diff_z, 123)
                            * easy3d::mat3::rotation(0, 0, -angle_z_orig)
                            * to_origin.normalize() * length(to_target);
            std::cout << "R * to_origin normalized * length target: " << next_vec << std::endl;
            std::cout << "next node calculated: " << coords_start + next_vec << std::endl;

            std::cout << "\norigin angle z: " << angle_z_orig / (M_PI / 180) << std::endl;
            std::cout << "origin angle y: " << angle_y_orig / (M_PI / 180) << std::endl;
            std::cout << "target angle z: " << angle_z_target / (M_PI / 180) << std::endl;
            std::cout << "target angle y: " << angle_y_target / (M_PI / 180) << std::endl;
            std::cout << "diff. angle z: " << angle_diff_z / (M_PI / 180) << std::endl;
            std::cout << "diff. angle y: " << angle_diff_y / (M_PI / 180) << std::endl;
        }

        // make sure angles very close to 0 or 360 degrees get outputted as 0
        if (abs(2 * M_PI - angle_diff_y) > 0.0001 && (abs(0 - angle_diff_y) > 0.0001)) {
            std::get<0>(movement) = angle_diff_y;
        }
        if (abs(2 * M_PI - angle_diff_z) > 0.0001 && (abs(0 - angle_diff_z) > 0.0001)) {
            std::get<1>(movement) = angle_diff_z;
        }
        std::get<2>(movement) = branch_length;
    }
    return movement;
}


void Lsystem::writeMovement(SGraphVertexDescriptor startV,
                            SGraphVertexDescriptor nextV,
                            Skeleton *skel,
                            int accuracy){
    // get relative movement between startV and nextV
    std::tuple<double, double, double> movement = moveToNext(startV, nextV, skel);
    // angles are rounded to int
    double angle_y = std::get<0>(movement);
    double angle_z = std::get<1>(movement);
    // option to output as degrees instead of radians
    if (degrees_){
        angle_y = std::get<0>(movement) / (M_PI / 180);
        angle_z = std::get<1>(movement) / (M_PI / 180);
    }
    double distance = std::get<2>(movement);
    if (grow_){
        //if (fast) distance = distance*(1+grow_sp*ratio);
        distance = distance*(1+grow_sp);
    }

    /// write rotation
    // rounded to [accuracy] decimals
    if (angle_y > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << angle_y;
        std::string angle_y_string = ss.str();
        Lstring_ += "+(" + angle_y_string + ")";
        graph_lsys[nextV].lstring["rotation"] += "+(" + angle_y_string + ")";
    }
    if (angle_y < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << abs(angle_y);
        std::string angle_y_string = ss.str();
        Lstring_ += "-(" + angle_y_string + ")";
        graph_lsys[nextV].lstring["rotation"] += "-(" + angle_y_string + ")";
    }
    /// write roll
    if (angle_z > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << angle_z;
        std::string angle_z_string = ss.str();
        Lstring_ += ">(" + angle_z_string + ")";
        graph_lsys[nextV].lstring["roll"] += ">(" + angle_z_string + ")";
    }
    if (angle_z < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << abs(angle_z);
        std::string angle_z_string = ss.str();
        Lstring_ += "<(" + angle_z_string + ")";
        graph_lsys[nextV].lstring["roll"] += "<(" + angle_z_string + ")";
    }
    /// write forward
    if (distance > 0) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy) << distance;
        std::string dist_string = ss.str();
        Lstring_ += "F(" + dist_string + ")";
        graph_lsys[nextV].lstring["forward"] += "F(" + dist_string + ")";
    }
}


double Lsystem::getZAngle(vec3 vec){
    vec3 xaxis = {1, 0, 0};
    double angle_z;

    // angle is dependent on what side of the x-axis (XY plane) the vector is
    if (vec.y < 0){
        angle_z = - acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }
    else{
        angle_z = acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }

    // zero angle returns nan, should be 0
    if (isnan(angle_z)) {
        angle_z = 0;
    }

    return angle_z;
}


double Lsystem::getYAngle(vec3 vec){
    vec3 xaxis = {1, 0, 0};
    double angle_y;

    // angle is dependent on what side of the x-axis (XZ plane) the vector is
    if (vec.z < 0) {
        angle_y = acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }
    else{
        angle_y = (2 * M_PI) - acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }

    // zero angle returns nan, should be 0
    if (isnan(angle_y)) {
        angle_y = 0;
    }

    return angle_y;
}


void Lsystem::outputLsys(const std::string& out_type, const std::string& path){
    if(out_type == "json"){
        lsysToJson(path);
    } else if (out_type == "txt"){
        lsysToText(path);
    }
}

// growth
void Lsystem::buildBranches(Skeleton *skel) {
    std::vector<size_t> vs;
    std::vector<BranchNode> nodes;
    std::map<size_t, BranchNode> pool;
    std::vector<std::vector<size_t>> branches;
    std::pair<SGraphVertexIterator, SGraphVertexIterator> vi = boost::vertices(skel->get_simplified_skeleton());

//    int count_leaf =0;

    for (auto vit = vi.first; vit != vi.second; ++vit){
        if (boost::degree(*vit, skel->get_simplified_skeleton())!=0) {
            vs.push_back(*vit);
            BranchNode temp;
            temp.degree = boost::degree(*vit, skel->get_simplified_skeleton());
            temp.pre = skel->get_simplified_skeleton()[*vit].nParent;
            temp.cVert = skel->get_simplified_skeleton()[*vit].cVert;
            if(notLeaf(*vit, skel)) {
                temp.nexts = findNext(*vit, skel);
            }
            nodes.push_back(temp);
        }
    }

    for (int i=0; i<vs.size(); ++i){
        pool.insert(std::make_pair(vs[i], nodes[i]));
    }
    std::vector<size_t> wait_list;
    wait_list.push_back(skel->get_root());
    while (!wait_list.empty()){
        size_t root_ = wait_list.back();
        std::vector<size_t> branch;
        branch.push_back(root_);

        size_t next_ = pool[root_].nexts[pool[root_].visit_time];
        pool[root_].visit_time+=1;
        if (pool[root_].degree-1 <= pool[root_].visit_time) {
            wait_list.pop_back();
        }
        while (notLeaf(next_, skel)){
            branch.push_back(next_);

            if (pool[next_].degree-2 > pool[next_].visit_time) {
                wait_list.push_back(next_);
            }

            pool[next_].visit_time += 1;
            next_ = pool[next_].nexts[pool[next_].visit_time-1];
        }
        // TODO: grow?
        branch.push_back(next_);
        pool[next_].visit_time += 1;

        for (int i=0; i<branch.size(); i++){
            if (node_pos.count(branch[i])!=0 && node_pos[branch[i]] < branch.size()-i-1) continue;

            else node_pos[branch[i]] = branch.size()-i-1;

        }
        branches.push_back(branch);
    }
}


bool Lsystem::sprout(int pos, SGraphVertexDescriptor vid, Skeleton *skel) {
    if (node_pos.count(vid)!=0 && boost::degree(vid, skel->get_simplified_skeleton())==2 &&
        skel->get_simplified_skeleton()[vid].nParent!=skel->get_root()){
        if (node_pos[vid]==pos) return true;
    }
    return false;
}


std::string Lsystem::selectRule(SGraphVertexDescriptor startV, SGraphVertexDescriptor nextV, Skeleton *skel) {
//    return "";
    std::tuple<double, double, double> movement = moveToNext(startV, nextV, skel);
    if (std::get<0>(movement)>0 && std::get<1>(movement)>0) return "A";
    else if (std::get<0>(movement)<0 && std::get<1>(movement)>0) return "B";
    else if (std::get<0>(movement)>0 && std::get<1>(movement)<0) return "C";
    else if (std::get<0>(movement)<0 && std::get<1>(movement)<0) return "D";
    return "";
}


std::vector<size_t> Lsystem::findNext(size_t vid, Skeleton *skel) {

    std::pair<Graph::out_edge_iterator, Graph::out_edge_iterator> outei = boost::out_edges(vid, skel->get_simplified_skeleton());
    std::vector<size_t> nexts_;
    for (auto eit = outei.first; eit!=outei.second; ++eit){
        if (boost::target(*eit, skel->get_simplified_skeleton())!=skel->get_simplified_skeleton()[vid].nParent) nexts_.push_back(boost::target(*eit, skel->get_simplified_skeleton()));
    }
    return nexts_;
}


bool Lsystem::notLeaf(size_t vid, Skeleton *skel) {
    if (skel->get_simplified_skeleton()[vid].nParent != vid && boost::degree(vid, skel->get_simplified_skeleton())==1) return false;
    return true;
}


void Lsystem::buildRules(Skeleton *skel, int accuracy) {
    for (auto it=node_pos.begin(); it!=node_pos.end(); ++it){
        if (it->second==1){
            std::tuple<double, double, double> movement = moveToNext(it->first, findNext(it->first, skel)[0], skel);
            double angle_y = std::get<0>(movement);
            double angle_z = std::get<1>(movement);
            // option to output as degrees instead of radians
            if (degrees_){
                angle_y = std::get<0>(movement) / (M_PI / 180);
                angle_z = std::get<1>(movement) / (M_PI / 180);
            }
            double distance = std::get<2>(movement);

            LBranch lbranch;

            if (angle_y > 0){
                lbranch.rotationSign = "+";
                lbranch.rotationDegree = angle_y;
            }
            if (angle_y < 0){
                lbranch.rotationSign = "-";
                lbranch.rotationDegree = angle_y;
            }
            // roll
            if (angle_z > 0){
                lbranch.rollSign = ">";
                lbranch.rollDegree = angle_z;
            }
            if (angle_z < 0){
                lbranch.rollSign = "<";
                lbranch.rollDegree = angle_z;
            }
            // forward
            if (distance > 0) {
                lbranch.distance = distance;
            }
            last_branches.push_back(lbranch);
        }
    }
    int count_rotation = 0;
    int count_roll = 0;
    double sum_rotation = 0.0;
    double avg_rotation = 0.0;
    double sum_roll = 0.0;
    double avg_roll = 0.0;
    int count_rotation_ = 0;
    int count_roll_ = 0;
    double sum_rotation_ = 0.0;
    double avg_rotation_ = 0.0;
    double sum_roll_ = 0.0;
    double avg_roll_ = 0.0;
    int count_dis = 0;
    double sum_dis = 0.0;
    double avg_dis = 0.0;

    for (auto lb:last_branches){
        if (lb.rotationSign=="+"){
            count_rotation++;
            sum_rotation += lb.rotationDegree;
        }
        else if (lb.rotationSign=="-"){
            count_rotation_++;
            sum_rotation_ += lb.rotationDegree;
        }
        if (lb.rollSign==">"){
            count_roll++;
            sum_roll += lb.rollDegree;
        }
        else if (lb.rollSign=="<"){
            count_roll_++;
            sum_roll_ += lb.rollDegree;
        }
        count_dis++;
        sum_dis += lb.distance;
    }

    if (count_rotation!=0) avg_rotation = sum_rotation/count_rotation;
    if (count_rotation_!=0) avg_rotation_ = sum_rotation_/count_rotation_;
    if (count_roll!=0) avg_roll = sum_roll/count_roll;
    if (count_roll_!=0) avg_roll_ = sum_roll_/count_roll_;
    avg_dis = sum_dis/count_dis;

    double grow_rotation, grow_rotation_, grow_roll, grow_roll_;
///*  A test:
    if (degrees_){
        if (avg_rotation+30>360) grow_rotation = avg_rotation-30;
        else grow_rotation = avg_rotation+30;
        if (avg_rotation_-30 < -360) grow_rotation_ = avg_rotation_+30;
        else grow_rotation_ = avg_rotation_-30;
        if (avg_roll+30>360) grow_roll = avg_roll-30;
        else grow_roll = avg_roll+30;
        if (avg_roll_-30< -360) grow_roll_ = avg_roll_+30;
        else grow_roll_ = avg_roll_-30;
    }

    else {
        if (avg_rotation+30*M_PI/180>2*M_PI) grow_rotation = avg_rotation-30*M_PI/180;
        else grow_rotation = avg_rotation+30*M_PI/180;
        if (avg_rotation_-30*M_PI/180< -2*M_PI) grow_rotation_ = avg_rotation_+30*M_PI/180;
        else grow_rotation_ = avg_rotation_+30*M_PI/180;
        if (avg_roll+30*M_PI/180>2*M_PI) grow_roll = avg_roll-30*M_PI/180;
        else grow_roll = avg_roll+30*M_PI/180;
        if (avg_roll_-30*M_PI/180< -2*M_PI) grow_roll_ = avg_roll_+30*M_PI/180;
        else grow_roll_ = avg_roll_-30*M_PI/180;
    }

    std::string r1, r2, r3, r4;
    std::stringstream s1, s2, s3, s4;

    s1 << "[+(";
    s1 << std::fixed << std::setprecision(accuracy) << grow_rotation;
    s1 << ")>(";
    s1 << std::fixed << std::setprecision(accuracy) << grow_roll;
    s1 << ")F(";
    s1 << std::fixed << std::setprecision(accuracy) << avg_dis;
    s1 << ")]";
    r1 = s1.str();

    s2 << "[-(";
    s2 << std::fixed << std::setprecision(accuracy) << std::abs(grow_rotation_);
    s2 << ")>(";
    s2 << std::fixed << std::setprecision(accuracy) << grow_roll;
    s2 << ")F(";
    s2 << std::fixed << std::setprecision(accuracy) << avg_dis;
    s2 << ")]";
    r2 = s2.str();

    s3 << "[+(";
    s3 << std::fixed << std::setprecision(accuracy) << grow_rotation;
    s3 << ")<(";
    s3 << std::fixed << std::setprecision(accuracy) << std::abs(grow_roll_);
    s3 << ")F(";
    s3 << std::fixed << std::setprecision(accuracy) << avg_dis;
    s3 << ")]";
    r3 = s3.str();

    s4 << "[-(";
    s4 << std::fixed << std::setprecision(accuracy) << std::abs(grow_rotation_);
    s4 << ")<(";
    s4 << std::fixed << std::setprecision(accuracy) << std::abs(grow_roll_);
    s4 << ")F(";
    s4 << std::fixed << std::setprecision(accuracy) << avg_dis;
    s4 << ")]";
    r4 = s4.str();

//    grow_rules.push_back(r1);
//    grow_rules.push_back(r2);
//    grow_rules.push_back(r3);
//    grow_rules.push_back(r4);
    rules.insert(std::make_pair("A", r1));
    rules.insert(std::make_pair("B", r2));
    rules.insert(std::make_pair("C", r3));
    rules.insert(std::make_pair("D", r4));
}
