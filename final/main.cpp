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
    cyclone::real cor = 0.5;

    bool useOctree = true;
    bool optimize = true;
}_appSettings;

const int BALL_COUNT = 2; //1024, 2048, 4096, 8192, 16384, 32768;
const int ROOM_SIZE = 10;
//const int MAX_COLLISIONS = pow(BALL_COUNT, 1.3);
const int MAX_COLLISIONS = 100000;
const int OCT_TREE_DIVISIONS = 3;
//const int HASHING_CELL_SIZE = 2.5;
const float HASHING_CELL_SIZE = ROOM_SIZE / pow(2, OCT_TREE_DIVISIONS);

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

void initSpheres(std::vector<cyclone::Collider*>& colliders)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(-ROOM_SIZE, ROOM_SIZE);
    std::uniform_int_distribution<int> rot(-360, 360);

    // create all the balls
    for (int i = 0; i < BALL_COUNT; i++)
    {
        cyclone::Vector3 pos(dist(gen), dist(gen), dist(gen));
        cyclone::Vector3 ori(rot(gen), rot(gen), rot(gen));

        cyclone::Rigidbody* newRigidbody = new cyclone::Rigidbody(pos);
        cyclone::SphereCollider* sphereCollider = new cyclone::SphereCollider(newRigidbody, cyclone::Matrix4(), 0.5);
        newRigidbody->setInertiaTensor(cyclone::getSphereInertiaTensor(newRigidbody->getMass(), sphereCollider->getRadius()));
        newRigidbody->setAcceleration({ 0, -9.81, 0 });
        newRigidbody->setOrientation(cyclone::eulerToQuaternion(ori));

        colliders.push_back(sphereCollider);
    }
}
void initWalls(std::vector<cyclone::Collider*>& walls)
{
    // create walls
    cyclone::PlaneCollider* floor = new cyclone::PlaneCollider(nullptr, -ROOM_SIZE, { 0,  1,  0 });
    cyclone::PlaneCollider* ceil = new cyclone::PlaneCollider(nullptr, -ROOM_SIZE, { 0, -1,  0 });
    cyclone::PlaneCollider* wall1 = new cyclone::PlaneCollider(nullptr, -ROOM_SIZE, { 1,  0,  0 });
    cyclone::PlaneCollider* wall2 = new cyclone::PlaneCollider(nullptr, -ROOM_SIZE, { -1,  0,  0 });
    cyclone::PlaneCollider* wall3 = new cyclone::PlaneCollider(nullptr, -ROOM_SIZE, { 0,  0,  1 });
    cyclone::PlaneCollider* wall4 = new cyclone::PlaneCollider(nullptr, -ROOM_SIZE, { 0,  0, -1 });

    // push back the walls
    walls.push_back(floor);
    walls.push_back(ceil);
    walls.push_back(wall1);
    walls.push_back(wall2);
    walls.push_back(wall3);
    walls.push_back(wall4);
}

void updateRigidbodys(const std::vector<cyclone::Collider*>& colliders, cyclone::real dt, const Camera3D& camera)
{
    // update rigidbody
    for (auto c : colliders)
    {
        cyclone::Collider* collider = c;
        // update rigidbody
        if (collider->hasRigidbody())
        {
            collider->getRigidbody()->integrate(dt);
        }
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
void checkOctreeCollisions(cyclone::OctreeElement* octree, cyclone::CollisionData* data)
{
    std::vector<std::pair<cyclone::Collider*, cyclone::Collider*>> pairs;
    octree->queryPossiblePairs(pairs);

    for (auto p : pairs)
    {
        // check for overlap
        bool hit = cyclone::CollisionTests::checkCollide(p.first, p.second, data);
        if (hit)
        {
            p.first->incrementOverlapCount();
            p.second->incrementOverlapCount();
        }
    }
}
void checkHashingCollisions(cyclone::SpacialHashing* hashing, cyclone::CollisionData* data)
{
    std::vector<std::pair<cyclone::Collider*, cyclone::Collider*>> pairs;
    hashing->queryPossiblePairs(pairs);

    for (auto p : pairs)
    {
        // check for overlap
        bool hit = cyclone::CollisionTests::checkCollide(p.first, p.second, data);
        if (hit)
        {
            p.first->incrementOverlapCount();
            p.second->incrementOverlapCount();
        }
    }
}
void checkWallCollisions(const std::vector<cyclone::Collider*>& colliders,
    const std::vector<cyclone::Collider*>& walls, cyclone::CollisionData* data)
{
    for (int i = 0; i < colliders.size(); i++)
    {
        for (int j = 0; j < walls.size(); j++)
        {
            // This is just for the walls as its a cube
            if (abs(colliders[i]->getAxis(3).x) >= ROOM_SIZE - colliders[i]->getBoundingLength() ||
                abs(colliders[i]->getAxis(3).y) >= ROOM_SIZE - colliders[i]->getBoundingLength() ||
                abs(colliders[i]->getAxis(3).z) >= ROOM_SIZE - colliders[i]->getBoundingLength())
            {
                // check for overlap
                bool hit = cyclone::CollisionTests::checkCollide(colliders[i], walls[j], data);
                if (hit)
                {
                    colliders[i]->incrementOverlapCount();
                    walls[j]->incrementOverlapCount();
                }
            }
        }
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

void resetPos(const std::vector<cyclone::Collider*>& colliders)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(-ROOM_SIZE, ROOM_SIZE);

    for (auto c : colliders)
    {
        if (!c->getRigidbody()) continue;

        cyclone::Vector3 pos(dist(gen), dist(gen), dist(gen));
        c->getRigidbody()->setPosition(pos);
        c->getRigidbody()->setLinearVelocity({ 0, 0, 0 });
    }
}
void launch(const std::vector<cyclone::Collider*>& colliders)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> mag(-5, 5);
    for (auto c : colliders)
    {
        if (!c->getRigidbody()) continue;

        cyclone::Vector3 vel(mag(gen), mag(gen), mag(gen));
        c->getRigidbody()->setLinearVelocity(vel);
    }
}

int main() {
    // Initialization \
    --------------------------------------------------------------------------------------

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "octing some trees");

    rlImGuiSetup(true);

    Camera3D camera = { 0 };
    initCamera(camera);

    // create models
    Model planeModel = LoadModelFromMesh(GenMeshPlane(ROOM_SIZE * 2, ROOM_SIZE * 2, 1, 1));

    // create variables for instanced drawing
    Texture2D ballTex = LoadTexture("assets/joebiden.jpeg");
    //Texture2D ballTex = LoadTexture("assets/beachball.jpeg");
    //Texture2D ballTex = LoadTexture("assets/happy.png");
    //Texture2D ballTex = LoadTexture("assets/sus.png");
    //Texture2D ballTex = LoadTexture("assets/cheese.png");

    Mesh sphereMesh = GenMeshSphere(0.5, 8, 8);
    Material sphereMaterial = LoadMaterialDefault();
    Shader shader = LoadShader("assets/sphereVert.glsl", "assets/sphereFrag.glsl");
    sphereMaterial.shader = shader;

    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(shader, "instanceTransform");
    sphereMaterial.maps[MATERIAL_MAP_DIFFUSE].texture = ballTex;

    // create vector of objects 
    std::vector<cyclone::Collider*> colliders;
    initSpheres(colliders);
    std::vector<cyclone::Collider*> walls;
    initWalls(walls);

    cyclone::ContactResolver* resolver = new cyclone::ContactResolver();
    //resolver->setCOR(_appSettings.cor);

    cyclone::CollisionData* data = new cyclone::CollisionData();
    data->setCoeffRestitution(_appSettings.cor); // this could cause problems but probably works fine
    data->init(MAX_COLLISIONS);

    // Create Octree
    cyclone::OctreeElement* octree = cyclone::OctreeElement::buildOctree(cyclone::Vector3(), ROOM_SIZE, OCT_TREE_DIVISIONS);
    cyclone::SpacialHashing* hashing = new cyclone::SpacialHashing(HASHING_CELL_SIZE);

    Debug::success() << "Application Started";

    Timer collisionTimer;
    Timer insertionTimer;

    while (!WindowShouldClose()) {
        //Seconds between previous frame and this one
        float deltaTime = GetFrameTime();
        handleInput(camera, deltaTime);

        updateRigidbodys(colliders, deltaTime, camera);

        updateColliders(colliders);
        updateColliders(walls);

        data->setToStart();

        if (!_appSettings.optimize)
        {
            insertionTimer.start();
            insertionTimer.stop();

            collisionTimer.start();
            checkCollisions(colliders, data);
            collisionTimer.stop();
        }
        else
        {
            if (_appSettings.useOctree)
            {
                // Clear the octree and insert
                insertionTimer.start();
                octree->clear();
                for (auto c : colliders)
                    octree->insert(c);
                insertionTimer.stop();

                collisionTimer.start();
                checkOctreeCollisions(octree, data);
                collisionTimer.stop();
            }
            else
            {
                // clear map and insert
                insertionTimer.start();
                hashing->clear();
                for (auto c : colliders)
                    hashing->insert(c);
                insertionTimer.stop();

                collisionTimer.start();
                checkHashingCollisions(hashing, data);
                collisionTimer.stop();
            }
        }

        // Bottleneck here (reduces by about 5 fps)
        checkWallCollisions(colliders, walls, data);

        resolver->resolveContacts(data->getArray(), data->getCount(), deltaTime);

        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(WHITE);

        Matrix* sphereMatricies = (Matrix*)MemAlloc(BALL_COUNT * sizeof(Matrix));
        for (int i = 0; i < BALL_COUNT; i++)
        {
            sphereMatricies[i] = ew::CTR(colliders[i]->getTransform());
        }

        drawInstanced(sphereMesh, sphereMaterial, sphereMatricies, BALL_COUNT);

        draw(walls, &planeModel);

        EndMode3D();

        //Outside of 3D mode, we can draw 2D things directly to the screen in pixel coordinates
        DrawFPS(GetScreenWidth() - 128, 16);

        MemFree(sphereMatricies);

        //ImGUI
        rlImGuiBegin();
        ImGui::Begin("SETTINGS");

        if (ImGui::Button("Launch", ImVec2(-1, 20)))
            launch(colliders);

        std::string ballText = "Balls: " + std::to_string(BALL_COUNT);
        ImGui::Text(ballText.c_str());

        std::string collisionText = "Collision Calculate: " + std::to_string(collisionTimer.getElapsedTime()) + "ms";
        std::string insertionText = "Insertion Calculate: " + std::to_string(insertionTimer.getElapsedTime()) + "ms";
        ImGui::Text(collisionText.c_str());
        ImGui::Text(insertionText.c_str());

        ImGui::Checkbox("Use Optimization Pattern", &_appSettings.optimize);
        ImGui::Checkbox("Use Octree", &_appSettings.useOctree);

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

    UnloadMaterial(sphereMaterial);
    UnloadTexture(ballTex);
    UnloadMesh(sphereMesh);
    //UnloadShader(shader);

    CloseWindow();

    Debug::warn() << "Application Closing";

    Debug::removeInstance();
}