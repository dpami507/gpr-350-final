#version 430

layout (local_size_x = 1024, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding=0) buffer ssbo0 { vec4 positions[]; };
layout(std430, binding=1) buffer ssbo1 { vec4 velocities[]; };
layout(std430, binding=2) buffer ssbo2 { vec4 accels[]; };
layout(std430, binding=3) buffer ssbo3 { vec4 forceAccums[]; };
layout(std430, binding=4) buffer ssbo4 { vec4 invMass[]; };

layout(std430, binding=5) buffer ssbo5 { vec4 angVels[]; };
layout(std430, binding=6) buffer ssbo6 { vec4 iitws[]; };

layout(location=0) uniform float deltaTime;

void main()
{
	uint index = 0;
}