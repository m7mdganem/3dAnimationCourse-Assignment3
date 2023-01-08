#include "BasicScene.h"
#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include <iomanip>

#include "igl/per_vertex_normals.h"
#include "igl/per_face_normals.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/edge_flaps.h"
#include "igl/loop.h"
#include "igl/upsample.h"
#include "igl/AABB.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/circulation.h"
#include "igl/edge_midpoints.h"
#include "igl/collapse_edge.h"
#include "igl/edge_collapse_is_valid.h"
#include "igl/write_triangle_mesh.h"

// #include "AutoMorphingModel.h"

using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create( "camera", fov, float(width) / height, near, far);
    
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

 
    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    
    auto material{ std::make_shared<Material>("material", program)}; // empty material
    auto material1{ std::make_shared<Material>("material", program1)}; // empty material
//    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());
 
    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    auto cylMesh{IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj")};
    auto cubeMesh{IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj")};
    sphere1 = Model::Create( "sphere",sphereMesh, material);    
    cube = Model::Create( "cube", cubeMesh, material);
    
    //Axis
    Eigen::MatrixXd vertices(6,3);
    vertices << -1,0,0,1,0,0,0,-1,0,0,1,0,0,0,-1,0,0,1;
    Eigen::MatrixXi faces(3,2);
    faces << 0,1,2,3,4,5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6,3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6,2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys",vertices,faces,vertexNormals,textureCoords);
    axis.push_back(Model::Create("axis",coordsys,material1));
    axis[0]->mode = 1;   
    axis[0]->Scale(4,Axis::XYZ);
    // axis[0]->lineWidth = 5;
    root->AddChild(axis[0]);
    float scaleFactor = 1; 
    cyls.push_back( Model::Create("cyl",cylMesh, material));
    cyls[0]->Scale(scaleFactor, Axis::Z);
    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    root->AddChild(cyls[0]);

    for(int i = 1; i < number_of_links; i++)
    { 
        cyls.push_back( Model::Create("cyl", cylMesh, material));
        cyls[i]->Scale(scaleFactor, Axis::Z);   
        cyls[i]->Translate(1.6f * scaleFactor, Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f*scaleFactor));
        cyls[i-1]->AddChild(cyls[i]);

        axis.push_back(Model::Create("axis", coordsys, material1));
        axis[i]->mode = 1;
        axis[i]->Scale(2, Axis::XYZ);
        cyls[i-1]->AddChild(axis[i]);
        axis[i]->Translate(0.8f * scaleFactor, Axis::Z);
    }
    cyls[0]->Translate({0, 0, 0.8f * scaleFactor});

    root->RotateByDegree(-90, Axis::X);

    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
      return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
    };
    autoCube = AutoMorphingModel::Create(*cube, morphFunc);

  
    sphere1->showWireframe = true;
    autoCube->Translate({-6,0,0});
    autoCube->Scale(1.5f);
    sphere1->Translate({5,0,0});

    autoCube->showWireframe = true;
    camera->Translate(22, Axis::Z);
    root->AddChild(sphere1);
//    root->AddChild(cyl);
    root->AddChild(autoCube);
    // points = Eigen::MatrixXd::Ones(1,3);
    // edges = Eigen::MatrixXd::Ones(1,3);
    // colors = Eigen::MatrixXd::Ones(1,3);
    
    // cyl->AddOverlay({points,edges,colors},true);
    cube->mode =1   ; 
    auto mesh = cube->GetMeshList();

    //autoCube->AddOverlay(points,edges,colors);
    // mesh[0]->data.push_back({V,F,V,E});
    int num_collapsed;

  // Function to reset original mesh and data structures
    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;
   // igl::read_triangle_mesh("data/cube.off",V,F);
    igl::edge_flaps(F,E,EMAP,EF,EI);
    std::cout<< "vertices: \n" << V <<std::endl;
    std::cout<< "faces: \n" << F <<std::endl;
    
    std::cout<< "edges: \n" << E.transpose() <<std::endl;
    std::cout<< "edges to faces: \n" << EF.transpose() <<std::endl;
    std::cout<< "faces to edges: \n "<< EMAP.transpose()<<std::endl;
    std::cout<< "edges indices: \n" << EI.transpose() <<std::endl;

}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0f);
    program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
//    cyl->Rotate(0.001f, Axis::Y);
    cube->Rotate(0.1f, Axis::XYZ);
    CCD();
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event

    if (action == GLFW_PRESS) { // default mouse button press behavior
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;

        // if (pickedModel)
        //     debug("found ", pickedModel->isPickable ? "pickable" : "non-pickable", " model at pos ", x, ", ", y, ": ",
        //           pickedModel->name, ", depth: ", pickedModelDepth);
        // else
        //     debug("found nothing at pos ", x, ", ", y);

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel)
            pickedToutAtPress = pickedModel->GetTout();
        else
            cameraToutAtPress = camera->GetTout();

        bool is_picked_link = false;
        for (int link_index = 0; link_index < number_of_links; link_index++)
        {
            if (pickedModel == cyls[link_index])
            {
                pickedIndex = link_index;
                is_picked_link = true;
                break;
            }
        }
        if (!is_picked_link)
        {
            pickedIndex = 0;
        }
    }
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    auto system = camera->GetRotation().transpose();
    if (pickedModel) {
        std::shared_ptr<Model> model_to_translate = pickedModel;
        if (pickedModel == cyls[pickedIndex])
        {
            model_to_translate = cyls[0];
        }
        model_to_translate->TranslateInSystem(system * model_to_translate->GetRotation(), { 0, 0, -float(yoffset) });
        pickedToutAtPress = model_to_translate->GetTout();
    } else {
        root->TranslateInSystem(system, { 0, 0, -float(yoffset) });
        //camera->TranslateInSystem(system, {0, 0, -float(yoffset)});
        //cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    if (dragging) {
        auto system = camera->GetRotation().transpose() * GetRotation();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel) {
            //pickedModel->SetTout(pickedToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
            {
                std::shared_ptr<Model> model_to_translate = pickedModel;
                if (pickedModel == cyls[pickedIndex])
                {
                    model_to_translate = cyls[0];
                }
                model_to_translate->TranslateInSystem(system * model_to_translate->GetRotation(), {-float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0});
            }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);
                RotateLinkIfPickedOrScene(camera->GetRotation().transpose(), float(yAtPress - y) / angleCoeff, 0.0f, 0.0f);
            }
        } else {
           // camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                root->TranslateInSystem(system, {-float(xAtPress - x) / moveCoeff/10.0f, float( yAtPress - y) / moveCoeff/10.0f, 0});
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                root->RotateInSystem(system, float(x - xAtPress) / 180.0f, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                root->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                root->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
        xAtPress =  x;
        yAtPress =  y;
    }
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            case GLFW_KEY_UP:
                RotateLinkIfPickedOrScene(system, -0.1f, 0.0f, 0.0f);
                break;
            case GLFW_KEY_DOWN:
                RotateLinkIfPickedOrScene(system, 0.1f, 0.0f, 0.0f);
                break;
            case GLFW_KEY_LEFT:
                cyls[pickedIndex]->RotateInSystem(system, 0.1f, Axis::Y);
                break;
            case GLFW_KEY_RIGHT:
                cyls[pickedIndex]->RotateInSystem(system, -0.1f, Axis::Y);
                break;
            case GLFW_KEY_W:
                camera->TranslateInSystem(system, {0, 0.1f, 0});
                break;
            case GLFW_KEY_S:
                camera->TranslateInSystem(system, {0, -0.1f, 0});
                break;
            case GLFW_KEY_A:
                camera->TranslateInSystem(system, {-0.1f, 0, 0});
                break;
            case GLFW_KEY_D:
                PrintDestinationPosition();
                break;
            case GLFW_KEY_B:
                camera->TranslateInSystem(system, {0, 0, 0.1f});
                break;
            case GLFW_KEY_F:
                camera->TranslateInSystem(system, {0, 0, -0.1f});
                break;
            case GLFW_KEY_1:
                if( pickedIndex > 0)
                  pickedIndex--;
                break;
            case GLFW_KEY_2:
                if(pickedIndex < cyls.size()-1)
                    pickedIndex++;
                break;
            case GLFW_KEY_3:
                if( tipIndex >= 0)
                {
                  if(tipIndex == cyls.size())
                    tipIndex--;
                  sphere1->Translate(GetSpherePos());
                  tipIndex--;
                }
                break;
            case GLFW_KEY_4:
                if(tipIndex < cyls.size())
                {
                    if(tipIndex < 0)
                      tipIndex++;
                    sphere1->Translate(GetSpherePos());
                    tipIndex++;
                }
                break;
            case GLFW_KEY_SPACE:
                ccd_on = !ccd_on;
                break;
            case GLFW_KEY_T:
                PrintLinksTipsPositions();
                break;
            case GLFW_KEY_N:
                PickNextLink();
                break;
            case GLFW_KEY_P:
                PrintRotationMatrices();
                break;
        }
    }
}

void BasicScene::PrintLinksTipsPositions()
{
    for (int link_index = 0; link_index < number_of_links; link_index++)
    {
        Eigen::Vector3f link_tip = GetLinkEdgePosition(link_index, 1);
        std::cout << "Link " << link_index << ":" << std::endl;
        std::cout << "\tx: " << std::setw(15) << link_tip(0) << std::setfill(' ') << std::endl;
        std::cout << "\ty: " << std::setw(15) << link_tip(1) << std::setfill(' ') << std::endl;
        std::cout << "\tz: " << std::setw(15) << link_tip(2) << std::setfill(' ') << std::endl;
    }
}

void BasicScene::PrintDestinationPosition()
{
    Eigen::Vector3f destination = GetDestinationPosition();
    std::cout << "Destination: " << std::endl;
    std::cout << "\tx: " << std::setw(15) << destination(0) << std::setfill(' ') << std::endl;
    std::cout << "\ty: " << std::setw(15) << destination(1) << std::setfill(' ') << std::endl;
    std::cout << "\tz: " << std::setw(15) << destination(2) << std::setfill(' ') << std::endl;
}

void BasicScene::PickNextLink()
{
    if (pickedModel != cyls[pickedIndex] || pickedIndex == number_of_links - 1)
    {
        pickedIndex = 0;
    }
    else
    {
        pickedIndex += 1;
    }

    pickedModel = cyls[pickedIndex];
}

void BasicScene::PrintRotationMatrices()
{
    if (pickedModel == cyls[pickedIndex])
    {
        std::cout << "Link " << pickedIndex << " rotation matrices:" << std::endl;

        Eigen::Vector3f euler_angles = pickedModel->GetRotation().eulerAngles(2, 0, 2);

        float phi = euler_angles(0);
        float theta = euler_angles(1);
        float gamma = euler_angles(2);

        Eigen::Matrix3f phi_matrix;
        Eigen::Matrix3f theta_matrix;
        Eigen::Matrix3f gamma_matrix;

        phi_matrix.row(0) = Eigen::Vector3f(cos(phi), -sin(phi), 0);
        phi_matrix.row(1) = Eigen::Vector3f(sin(phi), cos(phi), 0);
        phi_matrix.row(2) = Eigen::Vector3f(0, 0, 1);

        std::cout << "  Phi matrix: " << std::endl;
        PrintMatrix3f(phi_matrix);

        theta_matrix.row(0) = Eigen::Vector3f(1, 0, 0);
        theta_matrix.row(1) = Eigen::Vector3f(0, cos(theta), -sin(theta));
        theta_matrix.row(2) = Eigen::Vector3f(0, sin(theta), cos(theta));

        std::cout << "  Theta matrix: " << std::endl;
        PrintMatrix3f(theta_matrix);

        gamma_matrix.row(2) = Eigen::Vector3f(cos(gamma), -sin(gamma), 0);
        gamma_matrix.row(2) = Eigen::Vector3f(sin(gamma), cos(gamma), 0);
        gamma_matrix.row(2) = Eigen::Vector3f(0, 0, 1);

        std::cout << "  Gamma matrix: " << std::endl;
        PrintMatrix3f(gamma_matrix);

    }
    else
    {
        Eigen::Matrix3f scene_rotation = root->GetRotation();
        std::cout << "Scene Rotation Matrix: " << std::endl;
        PrintMatrix3f(scene_rotation);
    }
}

void BasicScene::PrintMatrix3f(Eigen::Matrix3f matrix)
{
    Eigen::Vector3f row = matrix.row(0);
    std::cout << "\t" << std::setw(15) << row(0) << std::setfill(' ') << std::setw(15) << row(1) << std::setfill(' ') << std::setw(15) << row(2) << std::setfill(' ') << std::endl;

    row = matrix.row(1);
    std::cout << "\t" << std::setw(15) << row(0) << std::setfill(' ') << std::setw(15) << row(1) << std::setfill(' ') << std::setw(15) << row(2) << std::setfill(' ') << std::endl;

    row = matrix.row(2);
    std::cout << "\t" << std::setw(15) << row(0) << std::setfill(' ') << std::setw(15) << row(1) << std::setfill(' ') << std::setw(15) << row(2) << std::setfill(' ') << std::endl;

}

Eigen::Vector3f BasicScene::GetSpherePos()
{
      Eigen::Vector3f l = Eigen::Vector3f(1.6f,0,0);
      Eigen::Vector3f res;
      res = cyls[tipIndex]->GetRotation()*l;   
      return res;  
}

void BasicScene::RotateLinkIfPickedOrScene(Eigen::Transpose<Eigen::Matrix3f> system, float x, float y, float z)
{
    if (pickedModel == cyls[pickedIndex])
    {        
        Eigen::Matrix3f link_rotation = pickedModel->GetRotation();
        Eigen::Vector3f euler_angles = link_rotation.eulerAngles(0, 1, 2);

        float alpha = euler_angles(0) + x;
        float beta = euler_angles(1) + y;
        float gamma = euler_angles(2) + z;

        Eigen::Matrix3f x_axis_rotation;
        Eigen::Matrix3f y_axis_rotation;
        Eigen::Matrix3f z_axis_rotation;

        x_axis_rotation.row(0) = Eigen::Vector3f(1, 0, 0);
        x_axis_rotation.row(1) = Eigen::Vector3f(0, cos(alpha), sin(alpha));
        x_axis_rotation.row(2) = Eigen::Vector3f(0, -sin(alpha), cos(alpha));

        y_axis_rotation.row(0) = Eigen::Vector3f(cos(beta), 0, -sin(beta));
        y_axis_rotation.row(1) = Eigen::Vector3f(0, 1, 0);
        y_axis_rotation.row(2) = Eigen::Vector3f(sin(beta), 0, cos(beta));

        z_axis_rotation.row(0) = Eigen::Vector3f(cos(gamma), sin(gamma), 0);
        z_axis_rotation.row(1) = Eigen::Vector3f(-sin(gamma), cos(gamma), 0);
        z_axis_rotation.row(2) = Eigen::Vector3f(0, 0, 1);

        Eigen::Matrix3f rotation_matrix = z_axis_rotation * y_axis_rotation * x_axis_rotation;

        if (x > 0.0f || x < 0.0f)
        {
            pickedModel->Rotate(rotation_matrix.transpose() * link_rotation.transpose());
        }
        else
        {
            pickedModel->Rotate(link_rotation.transpose() * rotation_matrix.transpose());
        }
    }
    else
    {
        root->RotateInSystem(system, x, Axis::X);
        root->RotateInSystem(system, y, Axis::Y);
        root->RotateInSystem(system, z, Axis::Z);
    }
}

Eigen::Vector3f BasicScene::GetDestinationPosition()
{
    return sphere1->GetAggregatedTransform().col(3).head(3);
}

Eigen::Vector3f BasicScene::GetLinkEdgePosition(int link_index, int direction)
{
    return GetLinkCenterPosition(link_index) + direction * cyls[link_index]->GetRotation() * Eigen::Vector3f(0, 0, link_length / 2.0f);
}

Eigen::Vector3f BasicScene::GetLinkCenterPosition(int link_index)
{
    return cyls[link_index]->GetAggregatedTransform().col(3).head(3);
}

void BasicScene::CCD()
{
    if (!ccd_on)
    {
        return;
    }

    Eigen::Vector3f destination_position = GetDestinationPosition();
    if ((destination_position - GetLinkEdgePosition(0, -1)).norm() > link_length * number_of_links)
    {
        std::cout << "cannot reach" << std::endl;
        ccd_on = false;
        return;
    }

    for (int link_index = number_of_links - 1; link_index > -1; link_index--)
    {
        Eigen::Vector3f last_link_end_position = GetLinkEdgePosition(number_of_links - 1, 1);
        float distance_from_goal = (destination_position - last_link_end_position).norm();
        if (distance_from_goal < delta)
        {
            std::cout << "goal reached, distance: " << distance_from_goal << std::endl;
            ccd_on = false;
            return;
        }

        Eigen::Vector3f R = GetLinkEdgePosition(link_index, -1);
        Eigen::Vector3f RE_normalized = (last_link_end_position - R).normalized();
        Eigen::Vector3f RD_normalized = (destination_position - R).normalized();

        float RE_RD_dot_product = RE_normalized.dot(RD_normalized);
        float angle = -1 < RE_RD_dot_product && RE_RD_dot_product < 1 ? acosf(RE_RD_dot_product) : RE_RD_dot_product > 1 ? acosf(1.0f) : acosf(-1.0f);

        Eigen::Vector3f normal = RE_normalized.cross(RD_normalized);

        Eigen::Vector3f rotation_vector = cyls[link_index]->GetRotation().transpose() * normal;
        cyls[link_index]->Rotate(angle / 60.0f, rotation_vector);
    }
}