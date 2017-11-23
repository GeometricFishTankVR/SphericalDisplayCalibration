#version 330 core
uniform vec2 point_vertice;

out vec3 color;

void main(){
		if( dot(gl_PointCoord-0.5f,gl_PointCoord-0.5f) < 0.25f 
			&& dot(point_vertice, point_vertice) > 0.f ) 
			color = vec3(1,1,1);
		else
			discard;
}