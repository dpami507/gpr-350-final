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
    float percentToDraw = 100;
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
        ew::UpdateFlyCamera(&camera, dt, 50);
    }
}
float GetRandomFloat(float from, float to) {
    return from + (to - from) * (float)GetRandomValue(0, INT_MAX) / INT_MAX;
}

int main() {
   // Initialization \
   --------------------------------------------------------------------------------------

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "final countdown?!");

    rlImGuiSetup(true);

    Camera3D camera = { 0 };
    initCamera(camera);

    rlSetClipPlanes(0.1, 10000);
    Mesh sphereMesh = GenMeshSphere(1, 8, 8);

    char* shaderCode = LoadFileText("assets/particleCompute.glsl");
    int shaderData = rlCompileShader(shaderCode, RL_COMPUTE_SHADER);
    int computeShader = rlLoadComputeShaderProgram(shaderData);
    UnloadFileText(shaderCode);

    Shader particleShader = LoadShader(
        "assets/particleVert.glsl",
        "assets/particleFrag.glsl"
    );
    Texture2D ballTex = LoadTexture("assets/vision.png");
    int ballTexLoc = GetShaderLocation(particleShader, "texture0");

    int numParticles = 1024 * 1000;
    Vector4* positions = (Vector4*)RL_MALLOC(sizeof(Vector4) * numParticles);
    Vector4* velocities = (Vector4*)RL_MALLOC(sizeof(Vector4) * numParticles);

    for (int i = 0; i < numParticles; i++)
    {
        float angle = GetRandomFloat(0, 6.28);
        float dist = GetRandomFloat(50, 1000);
        float height = GetRandomFloat(-25, 25);
        Vector3 position = {
            cos(angle)* dist,
            height,
            sin(angle)* dist,
        };
        positions[i] = Vector4{ position.x, position.y, position.z, 0 };
        Vector3 velocity = Vector3{ 0, 0, 0 };
        velocities[i] = Vector4{ velocity.x, velocity.y, velocity.z, 0 };
    }

    int ssbo0 = rlLoadShaderBuffer(numParticles * sizeof(Vector4), positions, RL_DYNAMIC_COPY);
    int ssbo1 = rlLoadShaderBuffer(numParticles * sizeof(Vector4), velocities, RL_DYNAMIC_COPY);
    int ssbo2 = rlLoadShaderBuffer(numParticles * sizeof(Vector4), positions, RL_DYNAMIC_COPY);

    int particleVAO = rlLoadVertexArray();
    rlEnableVertexArray(particleVAO);

    std::vector<Vector3> vertices;
    for (int i = 0; i < sphereMesh.vertexCount; i++)
    {
        Vector3 newVec;
        newVec.x = sphereMesh.vertices[i * 3];
        newVec.y = sphereMesh.vertices[i * 3 + 1];
        newVec.z = sphereMesh.vertices[i * 3 + 2];
        vertices.emplace_back(newVec);
    }

    // position
    rlLoadVertexBuffer(vertices.data(), vertices.size() * sizeof(Vector3), false);
    rlSetVertexAttribute(0, 3, RL_FLOAT, false, 0, 0);
    rlEnableVertexAttribute(0);

    // uv
    rlLoadVertexBuffer(sphereMesh.texcoords, sphereMesh.vertexCount * 2 * sizeof(float), false);
    rlSetVertexAttribute(1, 2, RL_FLOAT, false, 0, 0);
    rlEnableVertexAttribute(1);

    // normal
    rlLoadVertexBuffer(sphereMesh.normals, sphereMesh.vertexCount * 3 * sizeof(float), false);
    rlSetVertexAttribute(2, 3, RL_FLOAT, false, 0, 0);
    rlEnableVertexAttribute(2);

    rlDisableVertexArray();

    float time = 0;
    float timeScale = 1.;
    float particleScale = 1.0;

    Debug::success() << "Application Started";

    while (!WindowShouldClose()) {
        //Seconds between previous frame and this one
        float deltaTime = GetFrameTime();
        int numInstances = (int)((_appSettings.percentToDraw * 10) / 1000 * numParticles);
        handleInput(camera, deltaTime);

        time += deltaTime;

        { // Compute Pass.
            rlEnableShader(computeShader);

            // Set our parameters. The indices are set in the shader.
            rlSetUniform(0, &time, SHADER_UNIFORM_FLOAT, 1);
            rlSetUniform(1, &timeScale, SHADER_UNIFORM_FLOAT, 1);
            rlSetUniform(2, &deltaTime, SHADER_UNIFORM_FLOAT, 1);

            rlBindShaderBuffer(ssbo0, 0);
            rlBindShaderBuffer(ssbo1, 1);
            rlBindShaderBuffer(ssbo2, 2);

            // We have numParticles/1024 workGroups. Each workgroup has size 1024.
            rlComputeShaderDispatch(numParticles / 1024, 1, 1);
            rlDisableShader();
        }

        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        { // Render Pass.
            BeginMode3D(camera);

            ClearBackground(WHITE);
            DrawGrid(100, 1);

            rlEnableShader(particleShader.id);

            SetShaderValueTexture(particleShader, ballTexLoc, ballTex);

            // Because we use rlgl, we must take care of matrices ourselves.
            // We need to only pass the projection and view matrix.
            // These will be used to make the particle face the camera and such.
            Matrix projection = rlGetMatrixProjection();
            Matrix view = GetCameraMatrix(camera);

            SetShaderValueMatrix(particleShader, 0, projection);
            SetShaderValueMatrix(particleShader, 1, view);
            SetShaderValue(particleShader, 2, &particleScale, SHADER_UNIFORM_FLOAT);

            rlBindShaderBuffer(ssbo0, 0);
            rlBindShaderBuffer(ssbo1, 1);

            // Draw the particles. Instancing will duplicate the vertices.
            rlEnableVertexArray(particleVAO);
            rlDrawVertexArrayInstanced(0, sphereMesh.vertexCount, numInstances);
            rlDisableVertexArray();
            rlDisableShader();
        }
        EndMode3D();

        //Outside of 3D mode, we can draw 2D things directly to the screen in pixel coordinates
        DrawFPS(GetScreenWidth() - 128, 16);

        //ImGUI
        rlImGuiBegin();
        ImGui::Begin("SETTINGS");

        std::string particleCount = "Particles: " + std::to_string(numParticles);
        ImGui::Text(particleCount.c_str());

        ImGui::SliderFloat("Percentage to Draw", &_appSettings.percentToDraw, 0, 100);
        std::string drawnParticles = "Drawn: " + std::to_string((int)(numParticles * (_appSettings.percentToDraw / 100)));
        ImGui::Text(drawnParticles.c_str());

        ImGui::End();
        rlImGuiEnd();

        EndDrawing();
    }

    CloseWindow();

    Debug::warn() << "Application Closing";

    Debug::removeInstance();
}