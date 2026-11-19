#version 330

// Input vertex attributes
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;

// Input instance attributes (Raylib passes this automatically via DrawMeshInstanced)
in mat4 instanceTransform;

// Input uniform values
uniform mat4 mvp;

// Output vertex attributes (to fragment shader)
out vec2 fragTexCoord;
out vec3 fragNormal;

void main()
{
    fragTexCoord = vertexTexCoord;
    fragNormal = vertexNormal;

    // Multiply the vertex position by the instance matrix first, then the camera MVP
    gl_Position = mvp * instanceTransform * vec4(vertexPosition, 1.0);
}
