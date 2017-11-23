#version 330 core

layout(location = 0) in vec2 vertexPosition_modelspace;

void main(){
	gl_Position =  vec4(vertexPosition_modelspace, 0.f, 1.0);
}

