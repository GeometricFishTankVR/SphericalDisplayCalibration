/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef RENDER_GL_SHADER_HPP_
#define RENDER_GL_SHADER_HPP_

#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <GL/glew.h>
#include <glfw/glfw3.h>

namespace multi_proj_calib
{
	class Shader {
	public:

		Shader() : m_programID(0), m_num_uniforms(0)
		{}

		~Shader()
		{
			glDeleteProgram(m_programID);		
		}

		int init(const char * vertex_file_path, const char * fragment_file_path, int num_uniforms);

		void loadUniformLocation(const GLchar *uniform_name, int uniform_index) { m_uniformID[uniform_index] = glGetUniformLocation(m_programID, uniform_name); }

		GLuint getUniformID(unsigned int attrib_index) { return m_uniformID[attrib_index]; }

		void useShader() { glUseProgram(m_programID); }

	private:
		int loadShaders(const char * vertex_file_path, const char * fragment_file_path);

		GLuint m_programID;
		unsigned int m_num_uniforms;
		std::vector<GLuint> m_uniformID;
	};
}


#endif
