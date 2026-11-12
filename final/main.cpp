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

    // hidden values just for internal refrences in main
    cyclone::real sphereRadius = 0.5f;
    cyclone::real coeffRestitution = 0.5f;
    cyclone::real clickForce = 5.0f;
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

    std::vector<cyclone::Collider*> colliders;

    cyclone::Rigidbody sphereBody1 = cyclone::Rigidbody();
    cyclone::Rigidbody cubeBody1 = cyclone::Rigidbody();

    cyclone::SphereCollider sphereCollider1(&sphereBody1, cyclone::Matrix4(), _appSettings.sphereRadius);
    colliders.push_back(&sphereCollider1);
    cyclone::PlaneCollider planeCollider1(nullptr, 0, { 0.0f,0.0f,1.0f });
    colliders.push_back(&planeCollider1);
    cyclone::BoxCollider cubeCollider1(&cubeBody1, cyclone::Matrix4(), {1.f,1.f,1.f});
    colliders.push_back(&cubeCollider1);

    cyclone::CollisionData worldData = cyclone::CollisionData();
    cyclone::ContactResolver worldResolver = cyclone::ContactResolver();
    worldResolver.setCOR(_appSettings.coeffRestitution);
    worldData.init((colliders.size() * (colliders.size() - 1)) / 2);

    Model cubeModel = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    Model sphereModel = LoadModelFromMesh(GenMeshSphere(_appSettings.sphereRadius, 40, 40));
    Model planeModel = LoadModelFromMesh(GenMeshPlane(10.0f, 10.0f, 1, 1));

    Debug::success() << "Application Started";

    while (!WindowShouldClose()) {
        //Seconds between previous frame and this one
        float deltaTime = GetFrameTime();
        handleInput(camera, deltaTime);

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
            // loop through every sphere with a rigidbody and check if the ray hits it. currently needs a better method of matching the sphere bodies with the 
            for (int i = 0; i < colliders.size(); i++) {
                if (colliders[i]->getType() == cyclone::ColliderType::Plane || colliders[i]->getRigidbody() == nullptr) continue;
                // since GetRayCollisionSphere only uses a sphere's pos and radius to determine intersection, we can actually still use only one sphere model for everything 
                RayCollision rayHit = GetRayCollisionSphere(ray, ew::CTR(colliders[i]->getRigidbody()->getPos()), static_cast<cyclone::SphereCollider*>(colliders[i])->getRadius());
                if (rayHit.hit) {
                    colliders[i]->getRigidbody()->addForceAtPoint(ew::RTC(ray.direction) * _appSettings.clickForce, ew::RTC(rayHit.point));
                }
            }
        }

        // Collision stuff here
        // update rigidbody data
        for (int i = 0; i < colliders.size(); i++) {
            if (colliders[i]->getRigidbody() != nullptr) colliders[i]->getRigidbody()->integrate(deltaTime);
        }

        // update collision data
        for (int i = 0; i < colliders.size(); i++) {
            colliders[i]->updateInternals();
        }

        // resolve collision tests
        for (int i = 0; i < colliders.size(); i++) {
            for (int j = i + 1; j < colliders.size(); j++) {
                // just by calling the tests, the worldData will be filled with contacts to resolve
                cyclone::CollisionTests::checkCollide(colliders[i], colliders[j], &worldData);
            }
        }

        // resolve all contacts gained
        worldResolver.resolveContacts(worldData.getArray(), worldData.getCount(), deltaTime);

        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(WHITE);

        for (auto collider : colliders) {
            switch (collider->getType()) {
            case cyclone::ColliderType::Box:
                if(collider->hasRigidbody())
                    cubeModel.transform = ew::CTR(collider->getRigidbody()->getTransformMatrix());
                else 
                    cubeModel.transform = ew::CTR(collider->getTransform());
                DrawModel(cubeModel, { 0,0,0 }, 1.0f, GREEN);
                DrawModelWires(cubeModel, { 0,0,0 }, 1.0f, BLACK);
                break;
            case cyclone::ColliderType::Sphere:
                if (collider->hasRigidbody())
                    sphereModel.transform = ew::CTR(collider->getRigidbody()->getTransformMatrix());
                else
                    sphereModel.transform = ew::CTR(collider->getTransform());
                DrawModel(sphereModel, { 0,0,0 }, 1.0f, RED);
                DrawModelWires(sphereModel, { 0,0,0 }, 1.0f, BLACK);
                break;
            case cyclone::ColliderType::Plane:
                if (collider->hasRigidbody())
                    planeModel.transform = ew::CTR(collider->getRigidbody()->getTransformMatrix());
                else
                    planeModel.transform = ew::CTR(collider->getTransform());
                DrawModel(planeModel, { 0,0,0 }, 1.0f, BLUE);
                DrawModelWires(planeModel, { 0,0,0 }, 1.0f, BLACK);
                break;
            }
            
        }

        DrawGrid(100, 1);

        EndMode3D();

        //Outside of 3D mode, we can draw 2D things directly to the screen in pixel coordinates
        DrawFPS(GetScreenWidth() - 128, 16);

        //ImGUI
        rlImGuiBegin();
        ImGui::Begin("SETTINGS");

        ImGui::End();
        rlImGuiEnd();

        EndDrawing();
    }

    CloseWindow();

    Debug::warn() << "Application Closing";

    Debug::removeInstance();
}