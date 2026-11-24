#include <iostream>

#include <raylib.h>
#include <raymath.h> //Needed for most Raylib math functions
#include <cyclone/cyclone.h>
#include <imgui.h>
#include <rlImGui.h>
#include "rlgl.h"

#include <ew/camera.h>
#include <ew/conversions.h>

#include <engine/helpers.h>
#include <engine/Debug.h>
#include "engine/timer.h"

#include <random>
#include <chrono>

using namespace engine;

struct AppSettings {
    cyclone::real cor = 0.5;
    cyclone::real friction = 0.5f;

    bool useOctree = true;
    bool optimize = true;

    bool debugOn = false;
    float debugTimestep = 0.016f;
    int collisionIterations = 20;
}_appSettings;

const int BALL_COUNT = 10000; //1024, 2048, 4096, 8192, 16384, 32768;
const int ROOM_SIZE = 10;
const int MAX_COLLISIONS = 500000;
const int CELL_SIZE = 4;

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
        ew::UpdateFlyCamera(&camera, dt, 20);
    }
}

void draw(const std::vector<cyclone::Collider*>& colliders, Model* model)
{
    // draw
    for (auto c : colliders)
    {
        model->transform = ew::CTR(c->getTransform());
        DrawModel(*model, { 0, 0, 0 }, 1, (c->getType() == cyclone::ColliderType::Plane) ? GRAY : (c->getOverlapCount() > 0) ? RED : BLUE);
        DrawModelWires(*model, { 0, 0, 0 }, 1, BLACK);
    }
}
void drawInstanced(const Mesh& mesh, const Material& mat, const Matrix* transforms, int instances)
{
    DrawMeshInstanced(mesh, mat, transforms, instances);
}

int main() {
    // Initialization \
    --------------------------------------------------------------------------------------

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "octing some trees");

    Texture2D ballTex = LoadTexture("assets/joebiden.jpeg");
    //Texture2D ballTex = LoadTexture("assets/vision.png");

    Mesh sphereMesh = GenMeshSphere(0.5, 8, 8);
    Material sphereMaterial = LoadMaterialDefault();
    Shader shader = LoadShader("assets/sphereVert.glsl", "assets/sphereFrag.glsl");
    sphereMaterial.shader = shader;

    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(shader, "instanceTransform");
    sphereMaterial.maps[MATERIAL_MAP_DIFFUSE].texture = ballTex;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> rot(0, 6.28);
    std::uniform_real_distribution<float> dist(1, 150);
    std::uniform_real_distribution<float> height(-5, 5);

    // Optimization
    cyclone::SpacialHashing hashing(CELL_SIZE);

    // Create Planets
    std::vector<cyclone::SphereCollider> colliders;
    std::vector<cyclone::Rigidbody> rbs;

    for (int i = 0; i < BALL_COUNT; i++)
    {
        cyclone::Rigidbody newRb = cyclone::Rigidbody();
        rbs.emplace_back(newRb);
    }
    for (int i = 0; i < BALL_COUNT; i++)
    {
        cyclone::SphereCollider newCol(&rbs[i], cyclone::Matrix4(), 0.5);

        float angle = rot(gen);
        cyclone::Vector3 pos(cos(angle) * dist(gen), height(gen), sin(angle) * dist(gen));
        rbs[i].setPosition(pos);
        rbs[i].setLinearVelocity(-pos.unit() * 10);

        colliders.emplace_back(newCol);
    }

    rlImGuiSetup(true);

    Camera3D camera = { 0 };
    initCamera(camera);

    cyclone::ContactResolver* resolver = new cyclone::ContactResolver(_appSettings.collisionIterations, 0.01f, 0.01f);
    cyclone::CollisionData* data = new cyclone::CollisionData();
    data->setCoeffRestitution(_appSettings.cor); // this could cause problems but probably works fine
    data->setFriction(_appSettings.friction);
    data->init(MAX_COLLISIONS);

    Debug::success() << "Application Started";

    Timer collisionTimer;
    Timer insertionTimer;

    while (!WindowShouldClose()) {
        //Seconds between previous frame and this one
        float deltaTime = GetFrameTime();
        handleInput(camera, deltaTime);

        // if debug is on, we only update a frame when a button is pressed.
        if (_appSettings.debugOn) {
            // update rigidbody
            for (cyclone::Rigidbody& rb : rbs)
            {
                rb.integrate(deltaTime);
            }
            // update internals
            for (cyclone::Collider& c : colliders)
            {
                c.updateInternals();
            }
            data->setToStart();

            // clear map and insert
            insertionTimer.start();
            hashing.clear();
            for (cyclone::Collider& c : colliders)
            {
                hashing.insert(&c);
            }
            insertionTimer.stop();

            // query collisions
            collisionTimer.start();
            std::vector<std::pair<cyclone::Collider*, cyclone::Collider*>> pairs;
            hashing.queryPossiblePairs(pairs);
            for (auto& p : pairs)
            {
                // check for overlap
                bool hit = cyclone::CollisionTests::checkCollide(p.first, p.second, data);
                if (hit)
                {
                    p.first->incrementOverlapCount();
                    p.second->incrementOverlapCount();
                }
            }
            collisionTimer.stop();

            // resolve
            resolver->resolveContacts(data->getArray(), data->getCount(), deltaTime);
        }
        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(WHITE);

        Matrix* sphereMatricies = (Matrix*)MemAlloc(BALL_COUNT * sizeof(Matrix));
        for (int i = 0; i < BALL_COUNT; i++)
        {
            sphereMatricies[i] = ew::CTR(colliders[i].getTransform());
        }

        drawInstanced(sphereMesh, sphereMaterial, sphereMatricies, BALL_COUNT);

        EndMode3D();

        //Outside of 3D mode, we can draw 2D things directly to the screen in pixel coordinates
        DrawFPS(GetScreenWidth() - 128, 16);

        MemFree(sphereMatricies);

        //ImGUI
        rlImGuiBegin();
        ImGui::Begin("SETTINGS");

        std::string ballText = "Balls: " + std::to_string(BALL_COUNT);
        ImGui::Text(ballText.c_str());

        std::string collisionText = "Collision Calculate: " + std::to_string(collisionTimer.getElapsedTime()) + "ms";
        std::string insertionText = "Insertion Calculate: " + std::to_string(insertionTimer.getElapsedTime()) + "ms";
        ImGui::Text(collisionText.c_str());
        ImGui::Text(insertionText.c_str());

        // debug options I added to see if collisions were working correctly
        ImGui::Checkbox("Debug: ", &_appSettings.debugOn);
        ImGui::SliderFloat("Debug Timestep", &_appSettings.debugTimestep, 0.001f, 0.100f);
        if(ImGui::DragInt("Collision Iterations", &_appSettings.collisionIterations)) {
            resolver->setCollisionIterations(_appSettings.collisionIterations);
        }
        if (ImGui::Button("Next Frame")) {
            deltaTime = _appSettings.debugTimestep;
            // update rigidbody
            for (cyclone::Rigidbody& rb : rbs)
            {
                rb.integrate(deltaTime);
            }
            // update internals
            for (cyclone::Collider& c : colliders)
            {
                c.updateInternals();
            }
            data->setToStart();

            // clear map and insert
            insertionTimer.start();
            hashing.clear();
            for (cyclone::Collider& c : colliders)
            {
                hashing.insert(&c);
            }
            insertionTimer.stop();

            // query collisions
            collisionTimer.start();
            std::vector<std::pair<cyclone::Collider*, cyclone::Collider*>> pairs;
            hashing.queryPossiblePairs(pairs);
            for (auto& p : pairs)
            {
                // check for overlap
                bool hit = cyclone::CollisionTests::checkCollide(p.first, p.second, data);
                if (hit)
                {
                    p.first->incrementOverlapCount();
                    p.second->incrementOverlapCount();
                }
            }
            collisionTimer.stop();

            // resolve
            resolver->resolveContacts(data->getArray(), data->getCount(), deltaTime);
        
        }

        ImGui::End();
        rlImGuiEnd();

        EndDrawing();
    }

    data->cleanup();

    UnloadMaterial(sphereMaterial);
    UnloadTexture(ballTex);
    UnloadMesh(sphereMesh);
    //UnloadShader(shader);

    CloseWindow();

    Debug::warn() << "Application Closing";

    Debug::removeInstance();
}