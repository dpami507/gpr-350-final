#version 430

layout (local_size_x = 1024, local_size_y = 1, local_size_z = 1) in;

layout (std430, binding = 0) buffer ssbo0 {vec4 positions[];};
layout (std430, binding = 1) buffer ssbo1 {vec4 velocities[];};
layout (std430, binding = 2) buffer ssbo2 {vec4 startPositions[];};

layout(location=0) uniform float time;
layout(location=1) uniform float timeScale;
layout(location=2) uniform float deltaTime;

const float PI = 3.14159;

void main()
{
    uint index = gl_GlobalInvocationID.x;
    vec3 pos = positions[index].xyz;

    // We reset the position when time is exactly zero.
    if (time == 0) {
        pos = startPositions[index].xyz;
    }

    // Since our particles are in the (-1, 1) space
    // we transform the position to be suitable for the Lorenz system.
    pos = 30*pos + vec3(0, 0, 30);

    // This is the Lorenz system (See wikipedia). Velocity=dx/dt.
    vec3 vel = cross(normalize(positions[index].xyz), vec3(0, 1, 0));
    vel *= 1000;

    // Update and transform back to (-1, 1).
    pos += vel*deltaTime*timeScale;
    pos = (pos - vec3(0, 0, 30)) / 30;

    // Assign the new values to the buffers.
    positions[index].xyz = pos;
    velocities[index].xyz = vel;
}