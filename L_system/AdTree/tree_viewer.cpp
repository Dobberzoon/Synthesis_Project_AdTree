/*
*	Copyright (C) 2019 by
*       Shenglan Du (dushenglan940128@163.com)
*       Liangliang Nan (liangliang.nan@gmail.com)
*       3D Geoinformation, TU Delft, https://3d.bk.tudelft.nl
*
*	This file is part of AdTree, which implements the 3D tree
*   reconstruction method described in the following paper:
*   -------------------------------------------------------------------------------------
*       Shenglan Du, Roderik Lindenbergh, Hugo Ledoux, Jantien Stoter, and Liangliang Nan.
*       AdTree: Accurate, Detailed, and Automatic Modeling of Laser-Scanned Trees.
*       Remote Sensing. 2019, 11(18), 2074.
*   -------------------------------------------------------------------------------------
*   Please consider citing the above paper if you use the code/program (or part of it).
*
*	AdTree is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	AdTree is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/



#include "tree_viewer.h"
#include "skeleton.h"

#include "L-system.h"

#include <3rd_party/glfw/include/GLFW/glfw3.h>	// Include glfw3.h after our OpenGL definitions
#include <easy3d/viewer/drawable.h>
#include <easy3d/viewer/camera.h>
#include <easy3d/viewer/soft_shadow.h>
#include <easy3d/util/dialogs.h>
#include <easy3d/util/file_system.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/viewer/shader_program.h>
#include <easy3d/viewer/shader_manager.h>
#include <easy3d/viewer/setting.h>
#include <easy3d/algo/remove_duplication.h>

#include <iostream>

#include "AdTree/generalisation/lbranchGen.h"

using namespace easy3d;
//using namespace boost;

TreeViewer::TreeViewer()
    : ViewerImGui("AdTree")
    , skeleton_(nullptr)
{
    set_background_color(vec3(1, 1, 1));

    camera_->setUpVector(vec3(0, 0, 1));
    camera_->setViewDirection(vec3(-1, 0, 0));
    camera_->showEntireScene();

    shadow_ = new SoftShadow(camera());
    shadow_->set_sample_pattern(SoftShadow::SamplePattern(2));
    shadow_->set_darkness(0.3f);
    shadow_->set_softness(0.9f);
    shadow_->set_background_color(background_color_);

    std::cout << usage() << std::endl;
}


TreeViewer::~TreeViewer()
{

}


void TreeViewer::cleanup() {
    if (skeleton_)
        delete skeleton_;

    if (shadow_)
        delete shadow_;

    ViewerImGui::cleanup();
}


PointCloud* TreeViewer::cloud() const {
    if (models().empty())
        return nullptr;
    else
        return dynamic_cast<PointCloud*>(models()[0]);
}


SurfaceMesh* TreeViewer::branches() const {
    if (models().size() < 2)
        return nullptr;
    else
        return dynamic_cast<SurfaceMesh*>(models()[1]);
}


SurfaceMesh* TreeViewer::leaves() const {
    if (models().size() < 3)
        return nullptr;
    else
        return dynamic_cast<SurfaceMesh*>(models()[2]);
}


bool TreeViewer::key_press_event(int key, int modifiers)
{
    if (key == GLFW_KEY_P && modifiers == GLFW_MOD_SHIFT) {
        if (!cloud())
            return false;
        cloud()->set_visible(!cloud()->is_visible());
		return true;
	}

    else if (key == GLFW_KEY_G && modifiers == GLFW_MOD_SHIFT) {
        if (!cloud())
            return false;

		//shift the visibility of the graph drawable
        LinesDrawable* graph_drawable = cloud()->lines_drawable("graph");
        if (!graph_drawable)
            create_skeleton_drawable(ST_SIMPLIFIED);
        if (graph_drawable)
            graph_drawable->set_visible(!graph_drawable->is_visible());
		return true;
	}

	else if (key == GLFW_KEY_B && modifiers == GLFW_MOD_SHIFT)
	{
        if (!branches())
            return false;
        branches()->set_visible(!branches()->is_visible());
		return true;
	}

	else if (key == GLFW_KEY_L && modifiers == GLFW_MOD_SHIFT)
	{
        if (!leaves())
            return false;
        leaves()->set_visible(!leaves()->is_visible());
		return true;
	}

	else
		return Viewer::key_press_event(key, modifiers);
}


std::string TreeViewer::usage() const {
    return Viewer::usage() + std::string(
                "  Shift + P:       Show/Hide point cloud       \n"
                "  Shift + G:       Show/Hide skeleton          \n"
                "  Shift + B:       Show/Hide branches          \n"
                "  Shift + L:       Show/Hide leaves            \n"
    );
}


bool TreeViewer::open()
{
    for (auto m : models_)
        delete m;
    models_.clear();

    // set rules
    isLsystem = false;

    const std::vector<std::string> filetypes = {"*.xyz"};
    const std::string& file_name = FileDialog::open(filetypes, std::string(""));

    if (Viewer::open(file_name)) {
        set_title("AdTree - " + file_system::simple_name(cloud()->name()));
        fit_screen();
        return true;
    }
    return false;
}


bool TreeViewer::open_lsystem()
{
    // get file
    const std::vector<std::string> filetypes = {"*.json"};
    const std::vector<std::string>& file_names = FileDialog::open(filetypes, true, "");

    // if no path is chosen exit function
    if (file_names.empty()){return false;}

    std::ifstream input(file_names[0].c_str());
    if (input.fail()) {
        std::cerr << "could not open file \'" << file_names[0] << "\'" << std::endl;
        return false;
    }

    // set rules
    isLsystem = true;

    // clear loaded models
    for (auto m : models_)
        delete m;
    models_.clear();

    // set window title
    set_title("AdTree - " + file_system::simple_name(file_names[0]));

    // read l-system
    Turtle turtle;
    turtle.readFile(file_names[0]);

    // make cloud
    PointCloud* baseCloud = new PointCloud;
    baseCloud->set_name(file_names[0]);

    // populate cloud
    auto pointList = turtle.getStoredPoints();
    auto anchor = turtle.getAnchor();
    for (auto p: pointList){baseCloud->add_vertex(p - anchor);}

    // check if cloud is populated
    if (pointList.size() == 0){
        std::cerr << "could not create cloud" << std::endl;
        return false;
    }

    // create and set model of cloud
    create_drawables(baseCloud);
    Model* model = baseCloud;
    model->set_name(file_names[0]);
    add_model(model);
    fit_screen(model);

    easy3d::PointCloud::ModelProperty<easy3d::dvec3> prop = cloud()->add_model_property<dvec3>("translation");
    prop[0] = static_cast<dvec3> (turtle.getAnchor());
    std::cout << "tree origin has been translated by [" << -prop[0] << "]" << std::endl;
    std::cout << "cloud loaded. num vertices: " << cloud()->n_vertices() << std::endl;

    // create skeleton
    skeleton_ = new Skeleton;
    if (!skeleton_->clone_skeleton(turtle)) {return false;}
    create_skeleton_drawable(ST_SIMPLIFIED);

    // create mesh
    SurfaceMesh *mesh = new SurfaceMesh;
    mesh->set_name(file_names[0]);
    bool status =  skeleton_->reconstruct_mesh(cloud(), mesh);

    if (status) {
        auto offset = cloud()->get_model_property<dvec3>("translation");
        if (offset) {
            auto prop = mesh->model_property<dvec3>("translation");
            prop[0] = offset[0];
        }
        if (!branches())
            add_model(mesh);

        cloud()->set_visible(false);
        return true;
    }
    return false;
}


bool TreeViewer::save() const {
    SurfaceMesh* mesh = branches();
    if (!mesh) {
        std::cerr << "model of branches does not exist" << std::endl;
        return false;
    }

    const std::vector<std::string> filetypes = {"*.obj"};
    const std::string& file_name = FileDialog::save(filetypes, mesh->name());
    if (file_name.empty())
        return false;

    if (SurfaceMeshIO::save(file_name, mesh)) {
        std::cout << "successfully saved the model of branches to file" << std::endl;
        return true;
    }
    else {
        std::cerr << "failed saving the model of branches" << std::endl;
        return false;
    }
}


void TreeViewer::export_skeleton() const {
    if (!branches() || !skeleton_) {
        std::cerr << "model of skeleton does not exist" << std::endl;
        return;
    }

    const ::Graph& skeleton = skeleton_->get_simplified_skeleton();
    if (boost::num_edges(skeleton) == 0) {
        std::cerr << "skeleton has 0 edges" << std::endl;
        return;
    }

    const std::vector<std::string> filetypes = {"*.ply"};
    const std::string& initial_name = file_system::base_name(cloud()->name()) + "_skeleton.ply";
    const std::string& file_name = FileDialog::save(filetypes, initial_name);
    if (file_name.empty())
        return;

    // convert the boost graph to Graph (avoid modifying easy3d's GraphIO, or writing IO for boost graph)
    std::vector<vec3> vertices;
    std::vector<std::tuple<int, int>> edges;
    std::map<int,int> off_map;
    int off_value = 0;

    auto vts = boost::vertices(skeleton);
    for (SGraphVertexIterator iter = vts.first; iter != vts.second; ++iter) {
        int vd = *iter;
        if (boost::degree(vd, skeleton) != 0 ) { // ignore isolated vertices
            vertices.emplace_back(skeleton[vd].cVert);
            off_map.insert({vd, off_value});
        } else {
            off_value ++;
        }
    }

    auto egs = boost::edges(skeleton);
    for (SGraphEdgeIterator iter = egs.first; iter != egs.second; ++iter) {
        int s_b = boost::source(*iter, skeleton);
        int t_b = boost::target(*iter, skeleton);

        std::tuple<int,int> i = { s_b - off_map[s_b], t_b - off_map[t_b] };
        edges.emplace_back(i);
    }

    std::ofstream storageFile;
    storageFile.open(file_name);

    // write header
    storageFile << "ply" << std::endl;
    storageFile << "format ascii 1.0" << std::endl;
    storageFile << "element vertex " << vertices.size() << std::endl;
    storageFile << "property float x" << std::endl;
    storageFile << "property float y" << std::endl;
    storageFile << "property float z" << std::endl;
    storageFile << "element edge " << edges.size() << std::endl;
    storageFile << "property int vertex1" << std::endl;
    storageFile << "property int vertex2" << std::endl;
    storageFile << "end_header" << std::endl << std::endl;


    storageFile << std::setprecision(10); // allow for larger values being written to avoid rounding
    // very cheap fix
    if (isLsystem) {
        vec3 trans = skeleton_->getAnchor();
        for (auto &vertex : vertices) {
            storageFile << vertex + trans << std::endl;
        }
    } else {
        vec3 trans = skeleton_->get_translation();
        for (auto &vertex : vertices) {
            storageFile << vertex + trans << std::endl;
        }
    }

    storageFile << std::endl;
    for (const auto& edge: edges) {
        storageFile << std::get<0>(edge) << " " << std::get<1>(edge) << std::endl;
    }

    storageFile.close();
    std::cout << "skeleton file stored" <<std::endl;
}


void TreeViewer::export_leaves() const {
    SurfaceMesh* mesh = leaves();
    if (!mesh) {
        std::cerr << "model of leaves does not exist" << std::endl;
        return;
    }

    const std::vector<std::string> filetypes = {"*.obj"};
    const std::string& file_name = FileDialog::save(filetypes, mesh->name());
    if (file_name.empty())
        return;

    if (SurfaceMeshIO::save(file_name, mesh))
        std::cout << "successfully saved the model of leaves to file" << std::endl;
    else
        std::cerr << "failed saving the model of leaves" << std::endl;
}


bool TreeViewer::export_lsystem(bool deg,
                                bool gen,
                                bool grow,
                                int sprout_pos,
                                std::vector<float> species_info,
                                int steps_to_average) const{

    if (!branches() || !skeleton_) {
        std::cerr << "model of skeleton does not exist" << std::endl;
        return false;
    }

    const std::vector<std::string> filetypes = {"*.json", "*.txt"};
    const std::string& initial_name = file_system::base_name(cloud()->name()) + "_lsystem.json";
    const std::string& file_name = FileDialog::save(filetypes, initial_name);

    /// L-system initialization
    auto *lsys = new Lsystem();
    if (sprout_pos>lsys->sprout_pos) lsys->sprout_pos = sprout_pos;
    if (species_info[0]>lsys->grow_sp) lsys->grow_sp = species_info[0];
    if (species_info[1]>lsys->ratio) lsys->ratio = species_info[1];
    if (species_info[2]>lsys->grow_co) lsys->grow_co = species_info[2];
    lsys->readSkeleton(skeleton_, deg, grow);
    std::cout << "sprout postion: " << lsys->sprout_pos << ", ";
    std::cout << "basic branch growth speed: " << lsys->grow_sp << ", segments near tips growth speed: " <<
    lsys->ratio*lsys->grow_sp << ", thickness growth speed: " << lsys->grow_co*lsys->grow_sp << std::endl;
    //lsys->printLsystem();
    std::cout << "-------------------------------------------\n" << std::endl;

    if (gen){
        auto lbranch = new Lbranch(lsys);
        lbranch->build_branches();

        /// write rules for averaged tips of branches
        // todo: rule marker as a parameter
        std::string rule_marker = "X";

        std::vector<SGraphVertexDescriptor> current_step = lbranch->get_leaves();
        lbranch->average_branch(current_step, steps_to_average, rule_marker);

        /// write rules and axiom to L-system
        std::vector<size_t> rt;
        rt.push_back(lsys->get_root());

        // clear axiom before writing with rules
        lsys->axiom = "";
        lsys->rules = lbranch->get_rules();
        lbranch->branches_to_lsystem(lsys, rt);

        /// check new generalised lsystem
        // can be removed later, if it is too big for larger datasets
//        lsys->printLsystem();
    }

    lsys->outputLsys(file_system::extension(file_name), file_name);

    return true;
}

bool TreeViewer::export_city_json() const {
    if (!branches() || !skeleton_) {
        std::cerr << "model does not exist" << std::endl;
        return false;
    }
    const ::Graph& skeleton = skeleton_->get_simplified_skeleton();
    if (boost::num_edges(skeleton) == 0) {
        std::cerr << "skeleton has 0 edges" << std::endl;
        return false;
    }

    const std::vector<std::string> filetypes = {"*.json", "*.txt"};
    const std::string& initial_name = file_system::base_name(cloud()->name()) + "_City.json";
    const std::string& file_name = FileDialog::save(filetypes, initial_name);

    vec3 trans;
    // very cheap fix
    if (isLsystem) {
        trans = skeleton_->getAnchor();
    } else {
        trans = skeleton_->get_translation();
    }

    //store verts and edges
    // convert the boost graph to Graph (avoid modifying easy3d's GraphIO, or writing IO for boost graph)
    std::vector<std::vector<float>> vertices;
    std::vector<std::tuple<int, int>> edges;
    std::vector<float> radii;
    //std::vector<int> leaves;
    float rootR = 0.0;
    std::map<int,int> off_map;
    int off_value = 0;

    auto vts = boost::vertices(skeleton);
    for (SGraphVertexIterator iter = vts.first; iter != vts.second; ++iter) {
        int vd = *iter;
        if (boost::degree(vd, skeleton) != 0 ) { // ignore isolated vertices
            auto v = skeleton[vd].cVert + trans;
            auto v_f = {v.x, v.y, v.z};
            vertices.emplace_back(v_f);
            off_map.insert({vd, off_value});

        } else {
            off_value ++;
        }
    }

    auto egs = boost::edges(skeleton);
    for (SGraphEdgeIterator iter = egs.first; iter != egs.second; ++iter) {
        int s_b = boost::source(*iter, skeleton);
        int t_b = boost::target(*iter, skeleton);

        radii.emplace_back(skeleton[*iter].nRadius);

        std::tuple<int,int> i = { s_b - off_map[s_b], t_b - off_map[t_b] };
        edges.emplace_back(i);
    }

    int steps = 10;
    float max_radius = *max_element(std::begin(radii), std::end(radii));
    std::cout << max_radius << std::endl;
    float delta_radius = max_radius/steps;
    std::vector<float> classes;
    std::vector<int> values;


    nlohmann::json types;

    for (int l = 0; l < steps; ++l) {
        classes.emplace_back(delta_radius * ((float) l + 1));
        nlohmann::json temp_type;

        temp_type = {{"class", l + 1},
                     {"radius", delta_radius * ((float) l + 1)}};
        types.emplace_back(temp_type);
    }

    for (float r : radii) {
        if (r < classes[0]){
            values.emplace_back(0);
            continue;
        } else if (r >= classes[steps - 1]) {
            values.emplace_back(steps - 1);
            continue;
        }
        for (int m = 1; m < classes.size(); ++m) {
            if (r > classes[m - 1] && r < classes[m]) {
                values.emplace_back(m);
                break;
            }
        }

    }

    nlohmann::ordered_json j;
    nlohmann::ordered_json geometry;
    nlohmann::ordered_json semantics;

    nlohmann::ordered_json object;
    nlohmann::ordered_json cityobject;

    semantics["types"] = types;
    semantics["values"] = values;


    geometry["type"] = "MultiLineString";
    geometry["lod"] = 2;
    geometry["boundaries"] = edges;
    geometry["semantics"] = semantics;
//    geometry["geometry"] = 2;

    object["type"] = "SolitaryVegetationObject";
    std::vector<nlohmann::ordered_json> g = {geometry};
    object["geometry"] = g;

    cityobject["oneTree"] = object;

    j["type"] = "CityJSON";
    j["version"] = "1.0";

    j["CityObjects"] = cityobject;

//    j["geometry"] = {geometry};
    j["vertices"] = vertices;


    std::ofstream storageFile(file_name);
    storageFile << std::setw(4) << j << std::endl;
    storageFile.close();
    std::cout << "CityJSON has been written." << std::endl;
    return true;
}


void TreeViewer::draw() {
    if (!shadowing_enabled_) {
        Viewer::draw();
        return;
    }

    if (cloud()) {
        const mat4& MVP = camera_->modelViewProjectionMatrix();
        // camera position is defined in world coordinate system.
        const vec3& wCamPos = camera_->position();
        // it can also be computed as follows:
        //const vec3& wCamPos = invMV * vec4(0, 0, 0, 1);
        const mat4& MV = camera_->modelViewMatrix();
        const vec4& wLightPos = inverse(MV) * setting::light_position;

        if (cloud()->is_visible()) {
            ShaderProgram* program = program = ShaderManager::get_program("points_color");
            if (!program) {
                std::vector<ShaderProgram::Attribute> attributes;
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::POSITION, "vtx_position"));
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::COLOR, "vtx_color"));
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::NORMAL, "vtx_normal"));
                program = ShaderManager::create_program_from_files("points_color", attributes);
            }
            if (program) {
                program->bind();
                program->set_uniform("MVP", MVP);
                program->set_uniform("wLightPos", wLightPos);
                program->set_uniform("wCamPos", wCamPos);
                program->set_uniform("ssaoEnabled", false);
                for (auto m : models_) {
                    if (!m->is_visible())
                        continue;
                    for (auto d : m->points_drawables()) {
                        if (d->is_visible()) {
                            program->set_uniform("lighting", d->normal_buffer());
                            program->set_uniform("per_vertex_color", d->per_vertex_color() && d->color_buffer());
                            program->set_uniform("default_color", d->default_color());
                            d->draw(false);
                        }
                    }
                }
                program->release();
            }
        }

        LinesDrawable* graph_drawable = cloud()->lines_drawable("graph");
        if (graph_drawable && graph_drawable->is_visible()) {
            ShaderProgram* program = ShaderManager::get_program("lines_color");
            if (!program) {
                std::vector<ShaderProgram::Attribute> attributes;
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::POSITION, "vtx_position"));
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::COLOR, "vtx_color"));
                program = ShaderManager::create_program_from_files("lines_color", attributes);
            }
            if (program) {
                program->bind();
                program->set_uniform("MVP", MVP);
                program->set_uniform("per_vertex_color", false);
                program->set_uniform("default_color", graph_drawable->default_color());
                graph_drawable->draw(false);
                program->release();
            }
        }
    }

    std::vector<TrianglesDrawable*> surfaces;
    if (branches() && branches()->is_visible()) {
        for (auto d : branches()->triangles_drawables())
            surfaces.push_back(d);
    }
    if (leaves() && leaves()->is_visible()) {
        for (auto d : leaves()->triangles_drawables())
            surfaces.push_back(d);
    }
    shadow_->draw(surfaces);
}


bool TreeViewer::create_skeleton_drawable(SkeletonType type)
{
    if (!cloud())
        return false;

	//get the skeleton graph to be rendered
    const ::Graph* skeleton = nullptr;
    switch (type) {
    case ST_DELAUNAY:
        skeleton = &(skeleton_->get_delaunay());
        break;
    case ST_MST:
        skeleton = &(skeleton_->get_mst());
        break;
    case ST_SIMPLIFIED:
        skeleton = &(skeleton_->get_simplified_skeleton());
        break;
    case ST_SMOOTHED:
        skeleton = &(skeleton_->get_smoothed_skeleton());
        break;
    }
    if (!skeleton)
	{
        std::cout << "skeleton does not exist" << std::endl;
		return false;
	}

	//create the vertices vector for rendering
	std::vector<vec3> graph_points;
    std::pair<SGraphEdgeIterator, SGraphEdgeIterator> ep = edges(*skeleton);
	SGraphVertexDescriptor dVertex1, dVertex2;
	vec3 pVertex1, pVertex2;
	for (SGraphEdgeIterator eIter = ep.first; eIter != ep.second; ++eIter)
	{
        dVertex1 = source(*eIter, *skeleton);
        dVertex2 = target(*eIter, *skeleton);
        pVertex1 = (*skeleton)[dVertex1].cVert;
        pVertex2 = (*skeleton)[dVertex2].cVert;
        assert(!has_nan(pVertex1));
        assert(!has_nan(pVertex2));
		graph_points.push_back(pVertex1);
		graph_points.push_back(pVertex2);
	}

	//initialize the line drawable object;
    LinesDrawable* graph_drawable = cloud()->lines_drawable("graph");
	if (!graph_drawable)
        graph_drawable = cloud()->add_lines_drawable("graph");
	graph_drawable->update_vertex_buffer(graph_points);
	graph_drawable->set_per_vertex_color(false);
	graph_drawable->set_default_color(vec3(0.0f, 0.0f, 0.0f));

	return true;
}


bool TreeViewer::reconstruct_skeleton() {
#ifndef NDEBUG
        message_box("Performance hint!",
                    "You are runing a debug version of AdTree, which can be slow.\n"
                    "Please consider building and running AdTree in release mode.",
                    Type::warning,
                    Choice::ok
                    );
#endif

    if (!cloud()) {
        std::cout << "point cloud does not exist" << std::endl;
        return false;
    }

    if (isLsystem){
        std::cout << "non valid lsystem operation" << std::endl;
        return false;
    }

    /*{   // offer users the option to remove duplicated points
        int answer = message_box("Robustness hint!",
                                 "The point cloud may has duplicated points. Remove duplication "
                                 "can improve robustness. Would like to do so?",
                                 Type::warning,
                                 Choice::yes_no
        );
        if (answer == 1) {
            const float threshold = cloud()->bounding_box().diagonal() * 0.001f;
            const auto &points_to_remove = RemoveDuplication::apply(cloud(), threshold);
            for (auto v : points_to_remove)
                cloud()->delete_vertex(v);
            cloud()->garbage_collection();
            cloud()->points_drawable("vertices")->update_vertex_buffer(cloud()->points());
            std::cout << cloud()->vertices_size() << " points remained" << std::endl;
        }
    }*/

    if (skeleton_)
        delete skeleton_;
    skeleton_ = new Skeleton;

    SurfaceMesh* mesh = branches();
    if (mesh)
        mesh->clear();
    else {
        mesh = new SurfaceMesh;
        mesh->set_name(file_system::base_name(cloud()->name()) + "_branches.obj");
    }
//    bool status = skeleton_->reconstruct_branches(cloud(), mesh);
    bool status0 = skeleton_->reconstruct_skeleton(cloud(), mesh);
    bool status = skeleton_->reconstruct_mesh(cloud(), mesh);

    /// new: L-system part
    //Lsystem *lsys = new Lsystem();
    //lsys->readSkeleton(skeleton_);

    if (status) {
        auto offset = cloud()->get_model_property<dvec3>("translation");
        if (offset) {
            easy3d::vec3 translation = {(float) offset[0][0], (float) offset[0][1], (float) offset[0][2]};
            skeleton_->set_translation(translation);
            auto prop = mesh->model_property<dvec3>("translation");
            prop[0] = offset[0];
        }
        if (!branches())
            add_model(mesh);

        cloud()->set_visible(false);
        return true;
    }

    return false;
}


bool TreeViewer::add_leaves() {
    if (!skeleton_) {
        std::cout << "please generate skeleton first!" << std::endl;
        return false;
    }

    SurfaceMesh* mesh = leaves();
    if (mesh)
        mesh->clear();
    else {
        mesh = new SurfaceMesh;
        mesh->set_name(file_system::base_name(cloud()->name()) + "_leaves.obj");
    }

    if (skeleton_->reconstruct_leaves(mesh)) {
        if (!leaves())
            add_model(mesh);

        auto offset = cloud()->get_model_property<dvec3>("translation");
        if (offset) {
            auto prop = mesh->model_property<dvec3>("translation");
            prop[0] = offset[0];
        }
        TrianglesDrawable* leaves_drawable = mesh->triangles_drawable("surface");
        if (leaves_drawable) {
            leaves_drawable->set_per_vertex_color(false);
            leaves_drawable->set_default_color(vec3(0.50f, 0.83f, 0.20f));
        }

        return true;
    }

    return false;
}

