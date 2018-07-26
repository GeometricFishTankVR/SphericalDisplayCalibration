/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_RENDER_GL_H_
#define CALIB_RENDER_GL_H_

#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>

#include <GL/glew.h>
#include <glfw/glfw3.h>
#include <glm/glm.hpp>

#include "shader.hpp"
#include "global.h"

namespace multi_proj_calib
{	
	
	namespace render
	{
		enum RenderPattern { CIRCLE_GRID, LINE_GRID, ONE_CIRCLE, TEXTURE };

		class CalibRenderGL {

		public:

			static const int m_pattern_rows = 10;
			static const int m_pattern_cols = 8;

			CalibRenderGL(uint width = setting::proj_width, uint height = setting::proj_height, uint win_count = 1, uint frm_rate = 120)
				: m_width(width), m_height(height), m_num_window(win_count + 1) // always plus one for primary dbg monitor
			{
				int err;
				err = initGlfw(frm_rate, true);
				if (!err)
					initGlew();
				else
				{
					std::cout << std::endl;
					std::cout << "CalibRenderGL constructor fail to initialize GLFW" << std::endl;
				}
			}
			~CalibRenderGL()
			{
				cleanup();
				for (uint i = 0; i < m_num_window; i++)
				{
					glfwDestroyWindow(m_pwindow.at(i));
				}
				glfwTerminate();
			}
			
			void clearDisplay(uint win_index)
			{
				glfwMakeContextCurrent(m_pwindow.at(win_index));
				glClear(GL_COLOR_BUFFER_BIT);
			}

			void cleanup()
			{
				//shader
				m_calib_shader.~Shader();
				//vbo
				glDeleteBuffers(1, &m_pattern_vbo);
				for (uint i = 0; i < m_num_window; i++)
				{
					//vao
					glDeleteVertexArrays(1, &m_pattern_vao.at(i));
					//texture
					glDeleteTextures(1, &m_proj_tex.at(i));
				}
				//texture
				glDeleteTextures(1, &m_grid_tex);
			}
			
			void flushBuffer()
			{
				uint curr_window = 0;
				glfwPollEvents();
				while (curr_window < m_num_window)
				{
					glfwSwapBuffers(m_pwindow.at(curr_window));
					curr_window++;
				}
			}

			void flushBuffer(uint win_index)
			{
				glfwSwapInterval(1);
				glfwPollEvents();
				glfwSwapBuffers(m_pwindow.at(win_index));
			}

			void showWindow(uint win_index, bool show)
			{
				if (win_index < m_num_window)
				{
					if (show)
						glfwShowWindow(m_pwindow[win_index]);
					else
						glfwHideWindow(m_pwindow[win_index]);
				}
				else
					std::cout << "illegal window index" << std::endl;
			}

			void drawPattern(uint win_index);
			
			void loadPattern(RenderPattern pattern, float pattern_size = 50.0);
			
			void loadCalibTexture(float* pixel_coord, uint win_index);

			void getdPatternPixel(float& d_width, float& d_height)
			{
				d_width = m_d_row * (m_width - 1)/ 2.f;
				d_height = m_d_col * (m_height - 1) / 2.f;
			}

			void setCirclePos( const float& cir_x, const float& cir_y)
			{
				m_cir_center.x = cir_x;
				m_cir_center.y = cir_y;
			}

			void setCameraIntrin(const glm::vec2& fc, 
								 const glm::vec2& cc, 
								 const glm::vec3& kc,
								 const glm::vec2& pc)
			{
				m_fc = fc;
				m_cc = cc;
				m_kc = kc;
				m_pc = pc;
				m_cam_res = glm::vec2(setting::cam_width, setting::cam_height);
			}

			bool reSetWindowSequence(const std::vector<int>& window_order);

		private:
			int initGlfw(int frm_rate, bool max_screen);
			int initGlew();

			int createCircleGridVertices(glm::vec2* p_vertex);
			int createLineGridVertices(glm::vec2* p_vertex);
			int createOneCircleVertices(glm::vec2* p_vertex);
			int createQuadVertices(glm::vec2* p_vertex);

			void printGraphicInfo();
			
			GLuint loadDDSFile(std::string file);

			float m_d_row;
			float m_d_col;

			glm::vec2 m_fc;
			glm::vec2 m_cc;
			glm::vec3 m_kc;
			glm::vec2 m_pc;
			glm::vec2 m_cam_res; // camera resolution

			uint m_num_vertices;
			uint m_width;
			uint m_height;
			uint m_num_window;

			std::vector<GLFWwindow*> m_pwindow;
			RenderPattern m_pattern;

			Shader m_calib_shader;
			glm::vec2 m_cir_center;
			std::vector<GLuint> m_pattern_vao; // vao for each window, including the primary window
			GLuint m_pattern_vbo; //shared vbo
			GLuint m_grid_tex;	  //shared grid texture	
			std::vector<GLuint> m_proj_tex; // calib result as texture for each window, including the primary window
		};
	}
}

#endif // !CALIB_RENDER_GL_H_
