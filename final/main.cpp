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
}_appSettings;

const int COUNT = 100000;

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

    //Texture2D ballTex = LoadTexture("assets/joebiden.jpeg");
    Texture2D ballTex = LoadTexture("assets/vision.png");

    Mesh sphereMesh = GenMeshSphere(0.5, 8, 8);
    Material sphereMaterial = LoadMaterialDefault();
    Shader shader = LoadShader("assets/sphereVert.glsl", "assets/sphereFrag.glsl");
    sphereMaterial.shader = shader;

    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(shader, "instanceTransform");
    sphereMaterial.maps[MATERIAL_MAP_DIFFUSE].texture = ballTex;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> rot(0, 6.28);
    std::uniform_real_distribution<float> dist(100, 150);
    std::uniform_real_distribution<float> height(-10, 10);

    std::vector<cyclone::SphereCollider> colliders;
    for (int i = 0; i < COUNT; i++)
    {
        cyclone::Rigidbody* newRb = new cyclone::Rigidbody(); 
        cyclone::SphereCollider newCol(newRb, cyclone::Matrix4(), 0.5);

        float angle = rot(gen);
        cyclone::Vector3 pos(cos(angle) * dist(gen), height(gen), sin(angle) * dist(gen));
        newRb->setPosition(pos);

        colliders.emplace_back(newCol);
    }

    cyclone::CollisionData data;
    data.init(50000);
    cyclone::ContactResolver resolver;
    resolver.setCOR(0.95);

    cyclone::SpacialHashing hashing(3);

    rlImGuiSetup(true);

    Camera3D camera = { 0 };
    initCamera(camera);

    Debug::success() << "Application Started";

    Timer insertionTimer;
    Timer collideTimer;

    while (!WindowShouldClose()) {
        //Seconds between previous frame and this one
        float deltaTime = GetFrameTime();
        handleInput(camera, deltaTime);

        data.setToStart();
        hashing.clear();

        insertionTimer.start();
        for (cyclone::SphereCollider& c : colliders)
        {
            c.getRigidbody()->setLinearVelocity(c.getRigidbody()->getPos().unit().crossProduct({0, 1, 0}) * 10);
            c.getRigidbody()->integrate(deltaTime);
            c.updateInternals();
            hashing.insert(&c);
        }
        insertionTimer.stop();

        collideTimer.start();
        std::vector<std::pair<cyclone::Collider*, cyclone::Collider*>> pairs;
        hashing.queryPossiblePairs(pairs);
        for (int i = 0; i < pairs.size(); i++)
        {
            cyclone::CollisionTests::checkCollide(pairs[i].first, pairs[i].second, &data);
        }
        resolver.resolveContacts(data.getArray(), data.getCount(), deltaTime);
        collideTimer.stop();

        Matrix* sphereMatricies = (Matrix*)MemAlloc(COUNT * sizeof(Matrix));
        for (int i = 0; i < COUNT; i++)
        {
            sphereMatricies[i] = ew::CTR(colliders[i].getTransform());
        }

        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(WHITE);

        DrawMeshInstanced(sphereMesh, sphereMaterial, sphereMatricies, COUNT);
        MemFree(sphereMatricies);

        DrawGrid(100, 1);

        EndMode3D();

        //Outside of 3D mode, we can draw 2D things directly to the screen in pixel coordinates
        DrawFPS(GetScreenWidth() - 128, 16);

        //ImGUI
        rlImGuiBegin();
        ImGui::Begin("SETTINGS");

        std::string sphereCount = "Spheres: " + std::to_string(COUNT);
        ImGui::Text(sphereCount.c_str());

        std::string insert = "Insertion: " + std::to_string(insertionTimer.getElapsedTime()) + "ms";
        ImGui::Text(insert.c_str());

        std::string collision = "Collision: " + std::to_string(collideTimer.getElapsedTime()) + "ms";
        ImGui::Text(collision.c_str());

        ImGui::End();
        rlImGuiEnd();

        EndDrawing();
    }

    CloseWindow();

    Debug::warn() << "Application Closing";

    Debug::removeInstance();
}