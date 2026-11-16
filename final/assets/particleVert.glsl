#version 430

// This is the vertex position of the base particle!
// This is the vertex attribute set in the code, index 0.
layout (location=0) in vec3 vertexPosition;
layout (location=1) in vec2 vertexTexCoord;
layout (location=2) in vec3 vertexNormal;

// Input uniform values.
layout (location=0) uniform mat4 projectionMatrix;
layout (location=1) uniform mat4 viewMatrix;
layout (location=2) uniform float particleScale;

// The two buffers we will be reading from.
// We can write to them here but should not.
layout(std430, binding=0) buffer ssbo0 { vec4 positions[]; };

// We will only output color.
out vec3 fragNormal;
out vec2 fragTexCoord;

void main()
{
	vec3 position = positions[gl_InstanceID].xyz;

	fragTexCoord = vertexTexCoord;
	fragNormal   = vertexNormal;   // world-space normal, valid for uniform scale

	// Scale the mesh, then move it to the particle's world position.
	vec3 worldPos = vertexPosition * particleScale + position;

	// Apply the full view matrix, so the mesh is oriented in the world like any other object.
	gl_Position = projectionMatrix * viewMatrix * vec4(worldPos, 1.0);
}