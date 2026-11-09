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

    Debug::success() << "Application Started";

    while (!WindowShouldClose()) {
        //Seconds between previous frame and this one
        float deltaTime = GetFrameTime();
        handleInput(camera, deltaTime);

        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(WHITE);

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