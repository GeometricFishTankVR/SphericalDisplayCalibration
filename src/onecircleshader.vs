#version 330 core

layout(location = 0) in vec2 vertexPosition_modelspace;

uniform vec2 point_vertice;
uniform vec2 pRes;

void main(){

	gl_Position.x = point_vertice.x * 2/(pRes.x - 1) - 1.f;
	gl_Position.y = - point_vertice.y * 2/(pRes.y - 1) + 1.f;

	gl_Position.z = 0.f;
	gl_Position.w = 1.0f;
}

