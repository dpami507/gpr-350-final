#include <iostream>

#include <raylib.h>
#include <raymath.h> //Needed for most Raylib math functions
#include <cyclone/cyclone.h>
#include <imgui.h>
#include <rlImGui.h>

#include <ew/camera.h>
#include <ew/conversions.h>

#include <engine/helpers.h>
#include <engine/Debug.h>
#include "engine/timer.h"

#include <random>
#include <chrono>

using namespace engine;

struct AppSettings {

    cyclone::real mPlanetoidRadius = 1.5f;

}_appSettings;

void initCamera(Camera& camera)
{
    camera.position = { 0.0f, 10.0f, 15.0f };   // Camera position
    camera.target = { 0.0f, 0.0f, 0.0f };       // Camera looking at point
    camera.up = { 0.0f, 1.0f, 0.0f };           // Camera up vecto
    camera.fovy = 45.0f;                        // Camera vertical field-of-view in degrees
    camera.projection = CAMERA_PERSPECTIVE;     // Camera projection type - perspective vs orthographic
}
void handleInput(Camera& camera, cyclone::real dt)
{
    //Input
    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
        DisableCursor();
    }
    if (IsMouseButtonReleased(MOUSE_RIGHT_BUTTON)) {
        EnableCursor();
    }
    //Only allow movement if the cursor is hidden
    if (IsCursorHidden()) {
        ew::UpdateFlyCamera(&camera, dt);
    }
}

int main() {
   // Initialization \
   --------------------------------------------------------------------------------------

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "final countdown?!");

    rlImGuiSetup(true);

    Camera3D camera = { 0 };
    initCamera(camera);

    Model sphereModel = LoadModelFromMesh(GenMeshSphere(_appSettings.mPlanetoidRadius, 16, 16 ));

    cyclone::CollisionData ColData;
    ColData.init(32);

    cyclone::Rigidbody* DavidRigidBody = new cyclone::Rigidbody({ 0.0f, 0.0f, 20.0f }, { 0,5,0 }, { 0,-1,0 }, 10.0f);
    cyclone::SphereCollider* DavidCollider = new cyclone::SphereCollider(DavidRigidBody, cyclone::Matrix4(), _appSettings.mPlanetoidRadius);
    cyclone::GravityBody* PlanetDavid = new cyclone::GravityBody(DavidCollider);
    cyclone::Rigidbody* TetraRigidBody = new cyclone::Rigidbody({ 0.0f, 0.0f, -20.0f }, { 0,-5,0 }, { 0,1,0 }, 10.0f);
    cyclone::SphereCollider* TetraCollider = new cyclone::SphereCollider(TetraRigidBody, cyclone::Matrix4(), _appSettings.mPlanetoidRadius);
    cyclone::GravityBody* PlanetTetra = new cyclone::GravityBody(TetraCollider);

    std::vector<cyclone::GravityBody*> planets;

    planets.push_back(PlanetDavid);
    planets.push_back(PlanetTetra);

    Debug::success() << "Application Started";

    while (!WindowShouldClose()) {
        //Seconds between previous frame and this one
        float deltaTime = GetFrameTime();
        handleInput(camera, deltaTime);

        for (int i = 0; i < planets.size(); i++)
        {
            planets[i]->update();
            planets[i]->integrate(deltaTime);
        }

        ColData.setToStart();

        for (int i = 0; i < planets.size(); i++)
        {
            for (int j = i + 1; j < planets.size(); j++)
            {
                cyclone::GravityBody::calcGravForce(planets[i], planets[j]);
                cyclone::CollisionTests::checkCollide(planets[i]->getCollider(), planets[j]->getCollider(), &ColData);
            }
        }

        cyclone::ContactResolver::resolveContacts(ColData.getArray(), ColData.getCount(), deltaTime);

        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(WHITE);

        for (int i = 0; i < planets.size(); i++)
        {
            sphereModel.transform = ew::CTR(planets[i]->getTransform());
            DrawModel(sphereModel, { 0,0,0 }, 1.0f, BLUE);
            DrawModelWires(sphereModel, { 0,0,0 }, 1.0f, BLACK);
        }

        DrawGrid(100, 1);

        EndMode3D();

        //Outside of 3D mode, we can draw 2D things directly to the screen in pixel coordinates
        DrawFPS(GetScreenWidth() - 128, 16);

        //ImGUI
        rlImGuiBegin();
        ImGui::Begin("SETTINGS");

        ImGui::BeginDisabled();

        Vector3 ddp = ew::CTR(PlanetDavid->getAxis(3));
        Vector3 ddtp = ew::CTR(PlanetDavid->getCollider()->getRigidbody()->getPos());
        Vector3 ddv = ew::CTR(PlanetDavid->getCollider()->getRigidbody()->getLinearVelocity());
        ImGui::DragFloat3("David Pos", &ddp.x);
        ImGui::DragFloat3("David True Pos", &ddtp.x);
        ImGui::DragFloat3("David Vel", &ddv.x);

        ImGui::EndDisabled();

        ImGui::End();
        rlImGuiEnd();

        EndDrawing();
    }

    CloseWindow();

    Debug::warn() << "Application Closing";

    Debug::removeInstance();
}