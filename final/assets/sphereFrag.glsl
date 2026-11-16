#version 330

in vec2 fragTexCoord;
in vec3 fragNormal;

// Output color
out vec4 finalColor;

uniform sampler2D texture0;

void main()
{
    vec4 color = texture(texture0, fragTexCoord);
    
    finalColor = vec4(abs(fragNormal), 1.0);
    finalColor = color;
}
