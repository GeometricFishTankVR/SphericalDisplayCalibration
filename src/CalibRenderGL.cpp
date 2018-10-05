#include "CalibRenderGL.h"
#include <windows.h>

namespace multi_proj_calib
{
	using std::cout;
	using std::endl;
	namespace render
	{
		bool CalibRenderGL::reSetWindowSequence(const std::vector<int>& window_order)
		{
			/// check if the size of window order is out of range
			if (window_order.size() != m_num_window -1 || window_order.size() <= 0 )
			{
				cout << "CalibRenderGL::reSetWindowSequence(): window numbers do not match with projector number. Fail to reset window sequence. " << endl;
				return false;
			}
			
			/// check if the window order uniquely includes all projector ids
			std::vector<bool> proj_ids;
			proj_ids.resize(window_order.size(), false);
			for (int i = 0; i < window_order.size(); i++)
			{
				proj_ids.at(window_order[i]-1) = true;
			}
			if (std::find(proj_ids.begin(), proj_ids.end(), false) != proj_ids.end())
			{
				cout << "CalibRenderGL::reSetWindowSequence(): window_order does not match with existing projector ids. " << endl;
				return false;
			}

			/// swap windows
			std::vector<GLFWwindow*> temp_pwindow(m_pwindow.size());
			temp_pwindow = m_pwindow;

			for (int i = 1; i < m_pwindow.size(); i++) // total window = # of projectors + 1 primary window
			{
				int idx = window_order[i-1];  // prinary window has the idx of 0, projector 1 starts at the window idx of 1
				m_pwindow[idx] = temp_pwindow[i];
			}
			return true;
		}
		
		int CalibRenderGL::initGlfw(int frm_rate, bool max_screen)
		{
			if (!glfwInit())
			{
				cout << "CalibRenderGL::initGlfw(): Fail to initialize GLFW" << endl;
				return -1;
			}

			cout << endl;
			cout << "===== initialize GLFW =====" << endl;

			glfwWindowHint(GLFW_SAMPLES, 4);
			glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
			glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
			glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
			glfwWindowHint(GLFW_AUTO_ICONIFY, !max_screen);
			glfwWindowHint(GLFW_REFRESH_RATE, frm_rate);
			glfwWindowHint(GLFW_VISIBLE, false);

			//Initialize windows
			int mon_count(0);

			GLFWmonitor** monitors = glfwGetMonitors(&mon_count);
			GLFWmonitor* primary = glfwGetPrimaryMonitor(); // debug monitor

			cout << "Number of Monitors connected:" << mon_count << endl;
			cout << "Number of Windows to create:" << m_num_window << endl;

			m_pwindow.clear();

			//create windows
			int pri_index;
			for (int i = 0; i < mon_count; i++)
			{
				if (monitors[i] == primary)// primary monitor binds with the original context
				{
					pri_index = i;
					m_pwindow.push_back(glfwCreateWindow(m_width, m_height, "primary window", NULL, NULL));
					if (m_pwindow.back() == NULL)
					{
						cout << "CalibRenderGL::initGlfw(): Fail to create primary GLFW window" << endl;
						return -1;
					}
				}
			}

			for (uint i = 0; i < m_num_window; i++)
			{
				if (i != pri_index)
				{
					if (m_num_window > (uint) mon_count) // more windows requested than monitors actually connected: create all windows in primary
						m_pwindow.push_back(glfwCreateWindow(m_width, m_height, "projector window", NULL, *m_pwindow.cbegin()));
					else
					{
						if (m_num_window == mon_count) //maxmize all windows
							m_pwindow.push_back(glfwCreateWindow(m_width, m_height, "projector window", monitors[i], *m_pwindow.cbegin()));
						else
						{
							cout << "CalibRenderGL::initGlfw(): More monitors than windows: need to decide which monitor to connect" << endl;
							return -1;
						}
					}

					if (m_pwindow.back() == NULL)
					{
						cout << "CalibRenderGL::initGlfw(): Fail to create GLFW window" << endl;
						return -1;
					}
				}
			}

			if (m_num_window == m_pwindow.size()) //sanity check just in case
			{
				m_pattern_vao.assign(m_num_window, 0);
				m_proj_tex.assign(m_num_window, 0);
			}
			else
			{
				cout << "CalibRenderGL::initGlfw(): number of windows created not equal to windows requested: bug in window creation" << endl;
				return -1;
			}
			return 0;
		}


		int CalibRenderGL::initGlew()
		{
			glfwMakeContextCurrent(*m_pwindow.cbegin());
			glewExperimental = true;
			if (glewInit() != GLEW_OK) {
				cout << "CalibRenderGL::initGlew(): Fail to initialize GLEW" << endl;
				return -1;
			}

			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

			printGraphicInfo();

			return 0;
		}

		void CalibRenderGL::printGraphicInfo()
		{
			cout << "GL_RENDERER = " << glGetString(GL_RENDERER) << endl;
		}

		void CalibRenderGL::loadPattern(RenderPattern pattern, float size)
		{
			m_pattern = pattern;

			std::vector<glm::vec2> vertex_buffer;

			glfwMakeContextCurrent(*m_pwindow.cbegin()); // "primary" window which holds data

			//----- initialize vao, vbo -----
			if (pattern == CIRCLE_GRID)
			{
				vertex_buffer.resize(m_pattern_rows * m_pattern_cols);

				glPointSize(size);
				m_num_vertices = createCircleGridVertices(vertex_buffer.data());
				// circle shader 
				int err = m_calib_shader.init(file::src_path + "passthroughshader.vs", file::src_path + "circleshader.fs", 2);
				m_calib_shader.loadUniformLocation("point_vertice", 0);
				if (err != 0) cout << " CalibRenderGL::loadPattern(): Fail to initialize circle shader" << endl;
				m_cir_center = glm::vec2(1.f, 1.f);
			}
			else if (pattern == LINE_GRID)
			{
				/* create grid pattern */
				vertex_buffer.resize(m_pattern_rows * m_pattern_cols);

				glEnable(GL_BLEND);
				glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
				glEnable(GL_LINE_SMOOTH);
				glLineWidth(3.0);
				m_num_vertices = createLineGridVertices(vertex_buffer.data());
			}
			else if (pattern == ONE_CIRCLE)
			{
				vertex_buffer.resize(1);

				glPointSize(size);
				m_num_vertices = createOneCircleVertices(vertex_buffer.data());
				// one circle shader 
				int err = m_calib_shader.init(file::src_path + "onecircleshader.vs", file::src_path + "circleshader.fs", 2);
				if (err != 0) cout << " CalibRenderGL::loadPattern(): Fail to initialize one circle shader" << endl;
				m_calib_shader.loadUniformLocation("point_vertice", 0);
				m_calib_shader.loadUniformLocation("pRes", 1);

				m_cir_center = glm::vec2(0.f, 0.f);
			}
			else if (pattern == TEXTURE)
			{
				vertex_buffer.resize(6);

				m_num_vertices = createQuadVertices(vertex_buffer.data());
				// shader
				int err = m_calib_shader.init(file::src_path + "passthroughshader.vs", file::src_path + "projshader.fs", 8);
				if (err != 0) cout << " CalibRenderGL::loadPattern(): Fail to initialize projshader" << endl;
				m_calib_shader.loadUniformLocation("pattern_sampler", 0);
				m_calib_shader.loadUniformLocation("proj_sampler", 1);
				m_calib_shader.loadUniformLocation("fc", 2);
				m_calib_shader.loadUniformLocation("cc", 3);
				m_calib_shader.loadUniformLocation("kc", 4);
				m_calib_shader.loadUniformLocation("pc", 5);
				m_calib_shader.loadUniformLocation("cRes", 6);
				m_calib_shader.loadUniformLocation("pRes", 7);

				// load texture
				m_grid_tex = loadDDSFile(file::data_path+file::gridtexture_file);
			}
			//vao
			glGenVertexArrays(1, &m_pattern_vao[0]);
			glBindVertexArray(m_pattern_vao[0]);

			//vbo
			glGenBuffers(1, &m_pattern_vbo);
			glBindBuffer(GL_ARRAY_BUFFER, m_pattern_vbo);
			glBufferData(GL_ARRAY_BUFFER, sizeof(*vertex_buffer.data()) * m_num_vertices,
				vertex_buffer.data(), GL_DYNAMIC_DRAW);

			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

			//release binding
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);

			uint curr_window = 1;
			while (curr_window < m_num_window)
			{
				glfwMakeContextCurrent(m_pwindow.at(curr_window));

				//GLflags
				if (pattern == CIRCLE_GRID || pattern == ONE_CIRCLE)
					glPointSize(size);
				else if (pattern == LINE_GRID)
				{
					glEnable(GL_BLEND);
					glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
					glEnable(GL_LINE_SMOOTH);
					glLineWidth(3.0);
				}
				//bind vao and vbo for the rest of windows
				glGenVertexArrays(1, &m_pattern_vao[curr_window]);
				glBindVertexArray(m_pattern_vao[curr_window]);
				glBindBuffer(GL_ARRAY_BUFFER, m_pattern_vbo);
				glEnableVertexAttribArray(0);
				glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
				//release binding
				glBindBuffer(GL_ARRAY_BUFFER, 0);
				glBindVertexArray(0);
				curr_window++;
			}

		}

		void CalibRenderGL::loadCalibTexture(float* pixel_coord, uint win_index)
		{
			if (win_index < m_num_window)
			{
				glGenTextures(1, &m_proj_tex.at(win_index));

				glBindTexture(GL_TEXTURE_2D, m_proj_tex.at(win_index));

				// Give the BUFFER to OpenGL
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_width, m_height, 0, GL_RGBA, GL_FLOAT, pixel_coord);

				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
				glGenerateMipmap(GL_TEXTURE_2D);
			}
			else
			{
				cout << endl;
				cout << "CalibRenderGL::loadCalibTexture(): illegal window index" << endl;
			}
		}

		void CalibRenderGL::drawPattern(uint win_index)
		{
			if (win_index < m_num_window)
			{
				glfwMakeContextCurrent(m_pwindow.at(win_index));
				glClear(GL_COLOR_BUFFER_BIT);
				if (m_pattern == CIRCLE_GRID || m_pattern == ONE_CIRCLE)
				{
					m_calib_shader.useShader();
					glBindVertexArray(m_pattern_vao.at(win_index));
					glUniform2f(m_calib_shader.getUniformID(0), m_cir_center.x, m_cir_center.y);
					glUniform2f(m_calib_shader.getUniformID(1), (float)m_width, (float)m_height);
					glDrawArrays(GL_POINTS, 0, m_num_vertices);
				}
				else if (m_pattern == LINE_GRID)
				{
					glBindVertexArray(m_pattern_vao.at(win_index));
					glDrawArrays(GL_LINES, 0, m_num_vertices);
				}
				else if (m_pattern == TEXTURE)
				{
					m_calib_shader.useShader();
					glBindVertexArray(m_pattern_vao.at(win_index));					
					
					glActiveTexture(GL_TEXTURE0);
					glBindTexture(GL_TEXTURE_2D, m_grid_tex);
					glUniform1i(m_calib_shader.getUniformID(0), 0);
					
					glActiveTexture(GL_TEXTURE1);
					glBindTexture(GL_TEXTURE_2D, m_proj_tex.at(win_index));
					glUniform1i(m_calib_shader.getUniformID(1), 1);

					glUniform2fv(m_calib_shader.getUniformID(2), 1, &m_fc[0]);
					glUniform2fv(m_calib_shader.getUniformID(3), 1, &m_cc[0]);
					glUniform3fv(m_calib_shader.getUniformID(4), 1, &m_kc[0]);
					glUniform2fv(m_calib_shader.getUniformID(5), 1, &m_pc[0]);
					glUniform2fv(m_calib_shader.getUniformID(6), 1, &m_cam_res[0]); // camera resolution
					glUniform2f(m_calib_shader.getUniformID(7), (float)m_width, (float)m_height); // projector resolution

					glDrawArrays(GL_TRIANGLES, 0, m_num_vertices);
				}
			}
			else
				cout << "CalibRenderGL::drawPattern(): illegal window index" << endl;
		}

		int CalibRenderGL::createCircleGridVertices(glm::vec2* p_vertex)
		{
			m_d_row = 2.0f / (m_pattern_rows + 1);
			m_d_col = 2.0f / (m_pattern_cols + 1);

			for (int i = 0; i < m_pattern_cols; i++)
			{
				for (int j = 0; j < m_pattern_rows; j++)
				{
					p_vertex[i * m_pattern_rows + j].x = m_d_row * (j + 1) - 1.0f;
					p_vertex[i * m_pattern_rows + j].y = m_d_col * (i + 1) - 1.0f;
				}
			}
			return m_pattern_cols * m_pattern_rows;
		}

		int CalibRenderGL::createOneCircleVertices(glm::vec2* p_vertex)
		{
			p_vertex[0].x = 0.f;
			p_vertex[0].y = 0.f;
			return 1;
		}
		int CalibRenderGL::createQuadVertices(glm::vec2* p_vertex)
		{		
			// two triangles
			p_vertex[0].x = -1.f;
			p_vertex[0].y = -1.f;

			p_vertex[1].x = 1.f;
			p_vertex[1].y = -1.f;

			p_vertex[2].x = 1.f;
			p_vertex[2].y = 1.f;

			p_vertex[3].x = -1.f;
			p_vertex[3].y = -1.f;

			p_vertex[4].x = -1.f;
			p_vertex[4].y = 1.f;

			p_vertex[5].x = 1.f;
			p_vertex[5].y = 1.f;
			return 6;

			return 4;
		}
		int CalibRenderGL::createLineGridVertices(glm::vec2* p_vertex)
		{
			m_d_row = 2.0f / (m_pattern_rows + 1);
			m_d_col = 2.0f / (m_pattern_cols + 1);

			// horizontal lines
			for (int i = 0; i < m_pattern_cols; i++)
			{
				p_vertex[i * 2].y = m_d_col * (i + 1) - 1.0f;
				p_vertex[i * 2].x = 1.0f;
				p_vertex[i * 2 + 1].y = m_d_col * (i + 1) - 1.0f;
				p_vertex[i * 2 + 1].x = -1.0f;
			}
			int num_horzt_vertex = m_pattern_cols * 2;

			//vertical lines
			for (int i = 0; i < m_pattern_rows; i++)
			{
				p_vertex[num_horzt_vertex + i * 2].y = 1.0f;
				p_vertex[num_horzt_vertex + i * 2].x = m_d_row * (i + 1) - 1.0f;
				p_vertex[num_horzt_vertex + i * 2 + 1].y = -1.0f;
				p_vertex[num_horzt_vertex + i * 2 + 1].x = m_d_row * (i + 1) - 1.0f;
			}
			return (m_pattern_cols + m_pattern_rows) * 2;
		}

		/***** modified function from http://www.opengl-tutorial.org/ *****/
		GLuint CalibRenderGL::loadDDSFile(std::string file)
		{
			const int FOURCC_DXT1 = 0x31545844; // Equivalent to "DXT1" in ASCII
			const int FOURCC_DXT3 = 0x33545844; // Equivalent to "DXT3" in ASCII
			const int FOURCC_DXT5 = 0x35545844; // Equivalent to "DXT5" in ASCII

			unsigned char header[124];
			FILE *fp;

			fopen_s(&fp, file.data(), "rb");
			if (fp == NULL)
				throw std::runtime_error("CalibRenderGL::loadDDSFile() fails: can't find file " + file);

			/* verify the type of file */
			char filecode[4];
			fread(filecode, 1, 4, fp);
			if (strncmp(filecode, "DDS ", 4) != 0) {
				fclose(fp);
				return 0;
			}
			/* get the surface desc */
			fread(&header, 124, 1, fp);
			uint height = *(uint*)&(header[8]);
			uint width = *(uint*)&(header[12]);
			uint linearSize = *(uint*)&(header[16]);
			uint mipMapCount = *(uint*)&(header[24]);
			uint fourCC = *(uint*)&(header[80]);

			unsigned char * buffer;
			uint bufsize;

			bufsize = mipMapCount > 1 ? linearSize * 2 : linearSize;
			buffer = (unsigned char*)malloc(bufsize * sizeof(unsigned char));
			fread(buffer, 1, bufsize, fp);

			fclose(fp);

			uint components = (fourCC == FOURCC_DXT1) ? 3 : 4;
			uint format;
			switch (fourCC)
			{
			case FOURCC_DXT1:
				format = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT;
				break;
			case FOURCC_DXT3:
				format = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT;
				break;
			case FOURCC_DXT5:
				format = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
				break;
			default:
				free(buffer);
				return 0;
			}

			// Create one OpenGL texture
			GLuint textureID;
			glGenTextures(1, &textureID);

			// "Bind" the newly created texture : all future texture functions will modify this texture
			glBindTexture(GL_TEXTURE_2D, textureID);
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

			uint blockSize = (format == GL_COMPRESSED_RGBA_S3TC_DXT1_EXT) ? 8 : 16;
			uint offset = 0;

			/* load the mipmaps */
			for (uint level = 0; level < mipMapCount && (width || height); ++level)
			{
				uint size = ((width + 3) / 4)*((height + 3) / 4)*blockSize;
				glCompressedTexImage2D(GL_TEXTURE_2D, level, format, width, height,
					0, size, buffer + offset);

				offset += size;
				width /= 2;
				height /= 2;

				// Deal with Non-Power-Of-Two textures.
				if (width < 1) width = 1;
				if (height < 1) height = 1;
			}
			free(buffer);
			return textureID;
		}
	}
}
