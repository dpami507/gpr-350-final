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
    cyclone::real fixedTimeStep = 0.0167;
    int iterations = 100;
    cyclone::real damping = 0.95;
    cyclone::real groundFriction = 1.0;
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

void updateColliders(const std::vector<cyclone::Collider*>& colliders)
{
    for (auto c : colliders)
    {
        // update internals
        cyclone::Collider* collider = c;
        collider->updateInternals();
    }
}
void checkCollisions(const std::vector<cyclone::Collider*>& colliders, cyclone::CollisionData* data)
{
    for (int i = 0; i < colliders.size(); i++)
    {
        for (int j = i + 1; j < colliders.size(); j++)
        {
            // check for overlap
            bool hit = cyclone::CollisionTests::checkCollide(colliders[i], colliders[j], data);
            if (hit)
            {
                colliders[i]->incrementOverlapCount();
                colliders[j]->incrementOverlapCount();
            }
        }
    }
}

const float ROPE_SEGMENT_LENGTH = 0.1f;
const float ROPE_THICKNESS = 0.1f;
const float ROPE_SEGMENTS = 120;
const int MASS = 1;

int main() {
   // Initialization \
   --------------------------------------------------------------------------------------

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "octing some trees");

    rlImGuiSetup(true);

    Camera3D camera = { 0 };
    initCamera(camera);

    // create models
    Model planeModel = LoadModelFromMesh(GenMeshPlane(10, 10, 1, 1));
    Model sphereModel = LoadModelFromMesh(GenMeshSphere(0.5, 32, 32));
    Model ropeModel = LoadModelFromMesh(GenMeshSphere(ROPE_THICKNESS, 16, 16));

    std::vector<cyclone::Collider*> colliders;
    cyclone::PlaneCollider* floorCollider = new cyclone::PlaneCollider(nullptr, 0, {0, 1, 0 });
    cyclone::SphereCollider* sphereCollider = new cyclone::SphereCollider(nullptr, cyclone::Matrix4(), 0.5);
    colliders.push_back(floorCollider);
    colliders.push_back(sphereCollider);

    cyclone::Rope* rope = new cyclone::Rope(ROPE_SEGMENTS, ROPE_SEGMENT_LENGTH, ROPE_THICKNESS, _appSettings.damping, MASS, {0, 5, .1}, cyclone::Vector3::GRAVITY);

    cyclone::ContactResolver* resolver = new cyclone::ContactResolver();
    resolver->setCOR(0.95);
    cyclone::CollisionData* data = new cyclone::CollisionData();
    data->init(1000);

    Debug::success() << "Application Started";

    cyclone::real accumulator = 0;
    cyclone::VerletParticle* selected = nullptr;
    float dragDistance = 0;

    while (!WindowShouldClose()) {
        //Seconds between previous frame and this one
        float deltaTime = GetFrameTime();
        handleInput(camera, deltaTime);

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
        {
            Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
            for (cyclone::VerletParticle& p : rope->getParticles())
            {
                RayCollision col = GetRayCollisionSphere(ray, ew::CTR(p.mPosition), rope->getThickness());
                if (col.hit == true)
                {
                    selected = &p;
                    dragDistance = col.distance;
                }
            }
        }
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON))
        {
            if (selected)
            {
                Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);

                selected->mInverseMass = 0;
                selected->mPosition = ew::RTC(ray.position + ray.direction * dragDistance);
                selected->mPrevPosition = ew::RTC(ray.position + ray.direction * dragDistance);
            }
        }
        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON))
        {
            if (selected)
            {
                selected->mInverseMass = 1.0 / MASS;
                selected = nullptr;
            }
        }

        accumulator += deltaTime;
        if (accumulator >= _appSettings.fixedTimeStep)
        {
            rope->integrate(_appSettings.iterations, _appSettings.fixedTimeStep);
            accumulator -= _appSettings.fixedTimeStep;
        }

        updateColliders(colliders);
        
        data->setToStart();

        checkCollisions(colliders, data);
        for (auto c : colliders)
            rope->checkCollision(c, _appSettings.groundFriction);

        resolver->resolveContacts(data->getArray(), data->getCount(), deltaTime);

        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(WHITE);

        for (int i = 0; i < rope->getParticles().size() - 1; i++)
        {
            DrawCylinderEx(
                ew::CTR(rope->getParticles()[i].mPosition),
                ew::CTR(rope->getParticles()[i + 1].mPosition),
                rope->getThickness(), rope->getThickness(), 16, RED
            );
        }

        sphereModel.transform = ew::CTR(sphereCollider->getTransform());
        DrawModel(sphereModel, ew::CTR(sphereCollider->getAxis(3)), 1, GRAY);
        DrawModelWires(sphereModel, ew::CTR(sphereCollider->getAxis(3)), 1, BLACK);
        planeModel.transform = ew::CTR(floorCollider->getTransform());
        DrawModel(planeModel, ew::CTR(floorCollider->getAxis(3)), 1, WHITE);
        DrawModelWires(planeModel, ew::CTR(floorCollider->getAxis(3)), 1, BLACK);

        EndMode3D();

        //Outside of 3D mode, we can draw 2D things directly to the screen in pixel coordinates
        DrawFPS(GetScreenWidth() - 128, 16);

        //ImGUI
        rlImGuiBegin();
        ImGui::Begin("SETTINGS");

        ImGui::DragFloat("Fixed Time Step", &_appSettings.fixedTimeStep, 0.01, 0.005, 0.1);
        ImGui::DragInt("Iterations", &_appSettings.iterations, 1, 1, 100);
        if (ImGui::DragFloat("Damping", &_appSettings.damping, 0.01, 0, 1))
            rope->setDamping(_appSettings.damping);
            
        ImGui::DragFloat("Ground Friction", &_appSettings.groundFriction, 0.01, 0, 1);

        if (ImGui::Button("Reset", ImVec2(-1, 20)))
            rope->setPosition({ 0, 5, 0 });

        ImGui::End();
        rlImGuiEnd();

        EndDrawing();
    }

    // Cleanup
    for (auto c : colliders)
    {
        if (c->getRigidbody() != nullptr)
            delete c->getRigidbody();

        delete c;
    }

    data->cleanup();

    UnloadModel(planeModel);
    UnloadModel(sphereModel);

    CloseWindow();

    Debug::warn() << "Application Closing";

    Debug::removeInstance();
}