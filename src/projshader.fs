#version 330 core

out vec3 color;

uniform sampler2D pattern_sampler;
uniform sampler2D proj_sampler;

uniform vec2 fc;
uniform vec2 cc;
uniform vec3 kc;
uniform vec2 pc;
uniform vec2 cRes;
uniform vec2 pRes;

void main(){
 
	float pwidth = pRes.x;
	float pheight = pRes.y;

	float cwidth = cRes.x;
	float cheight = cRes.y;

	float U = (gl_FragCoord.x - 0.5)/(pwidth - 1.0);
    float V = (gl_FragCoord.y - 0.5)/(pheight - 1.0);     
	
	vec3 proj_coord = texture(proj_sampler, vec2(U, 1.0 - V) ).xyz;
	float alpha = texture(proj_sampler, vec2(U, 1.0 - V) ).w;
	
	// lens distortions	
	float a = proj_coord.x / proj_coord.z;
	float b = proj_coord.y / proj_coord.z;
	float r2 = a*a + b*b;
	float r4 = r2 * r2;
	float r6 = r2 * r4;
	float xx = a*(1 + kc.x*r2 + kc.y*r4 + kc.z*r6) + 2*pc.x*a*b + pc.y*(r2 + 2*a*a);
    float yy = b*(1 + kc.x*r2 + kc.y*r4 + kc.z*r6) + pc.x*(r2 + 2*b*b) + 2*pc.y*a*b;

	vec2 rendered_tex_ind;
	
	rendered_tex_ind.x = xx * fc.x + cc.x;
	rendered_tex_ind.y = yy * fc.y + cc.y;

	vec2 rendered_tex_ind_uv;
	rendered_tex_ind_uv.x = rendered_tex_ind.x/(cwidth - 1.0);
	rendered_tex_ind_uv.y = rendered_tex_ind.y/(cheight - 1.0);

	if( rendered_tex_ind_uv.x >=0 && rendered_tex_ind_uv.y >=0 && rendered_tex_ind_uv.x <=1.0 && rendered_tex_ind_uv.y <= 1.0)
	{
		color.xyz = texture(pattern_sampler, rendered_tex_ind_uv).xyz ;
		color = color * alpha;
	}
	else
		discard;
}