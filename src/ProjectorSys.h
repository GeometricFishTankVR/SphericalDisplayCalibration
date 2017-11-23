/*
 * Automatic Calibration Approach For Spherical Multi-projector Display
 * Author: Qian Zhou (qzhou{at}ece.ubc.ca)
 * (c) University of British Columbia 2017.
 * Usage subject to the accompanying file "License.txt"
 */

#ifndef CALIB_PROJECJOR_H_
#define CALIB_PROJECTOR_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "CalibRenderGL.h"
#include "global.h"

namespace multi_proj_calib {
	enum Projector {PROJECTOR_1 = 1, PROJECTOR_2, PROJECTOR_3, PROJECTOR_4};
	
	/* DisplayCalibration.h: projector display class that communicates between DisplayCalibration class and CalibRenderGL class. CalibRenderGL class can be substituted by other Render classes that uses different Graphic Library. */
	
	class ProjectorSys
	{
	public:
		ProjectorSys(int projector_count = 1, int width = setting::proj_width, int height = setting::proj_height)
			: m_render(width, height, projector_count), m_num_proj(projector_count), m_width(width), m_height(height)
		{}
		~ProjectorSys() {}

		bool setProjectorSequence(const std::vector<int>& proj_order)
		{
			return m_render.reSetWindowSequence(proj_order);
		}

		//! load pattern 
		void initPattern(render::RenderPattern pattern)
		{
			m_render.loadPattern(pattern, 40);
		}

		//! clear pattern
		void clearProj(Projector proj)
		{
			m_render.clearDisplay(proj);
		}

		//! start to project: call before loop
		void startProj(Projector proj) 
		{
			if ((unsigned int) proj <= m_num_proj)
				m_render.showWindow(proj, true);
		}

		//! stop to project: call after loop
		void stopProj(Projector proj) 
		{
			if ((unsigned int)proj <= m_num_proj)
				m_render.showWindow(proj, false);
		}
		
		//! project pattern: call in loop
		void projPattern(Projector proj) 
		{
			if ((unsigned int)proj <= m_num_proj)
			{
					m_render.drawPattern(proj);
			}
		}

		//! swap render buffers: call in loop after projPattern, specifical for OpenGL
		void projFlush() 
		{
			m_render.flushBuffer();
		}

		void projFlush(Projector proj)
		{
			m_render.flushBuffer(proj);
		}

		//! load and convert calib result from DisplayCalibration class: pixel_pt for geometric result and alpha_mask for photometric result. Results are converted into TextureRGBA. 
		void loadCalibResult(std::vector<cv::Point3f>& pixel_pt, cv::Mat_<float> alpha_mask, Projector proj)
		{
			// merge geometric and photometric result
			// convert openCV data type to generic array
			size_t size = pixel_pt.size();
			std::vector<float> texture_rgba(size * 4);	// OpenGL Texture in the format of RGBA
			for (unsigned int i = 0; i < size; i++)
			{
				texture_rgba[4 * i] = pixel_pt[i].x;
				texture_rgba[4 * i + 1] = pixel_pt[i].y;
				texture_rgba[4 * i + 2] = pixel_pt[i].z;
				int idx_x = i / setting::proj_width;
				int idx_y = i % setting::proj_width;
				texture_rgba[4 * i + 3] = std::powf(alpha_mask.at<float>(idx_x, idx_y), 1/2.2f);

			}

			float* p_float = texture_rgba.data();
			m_render.loadCalibTexture(p_float, (unsigned int)proj);
		}

		//! clean up renderer stuff (vao, vbo, shader, texture): call in the end
		void cleanRender()
		{
			m_render.cleanup();
		}

		void getPatternSize(int& width, int& height)
		{
			width = m_render.m_pattern_rows;
			height = m_render.m_pattern_cols;
		}

		void getPatternDistancePixel(float& dwidth, float& dheight)
		{
			m_render.getdPatternPixel(dwidth, dheight);
		}

		//! send blob center position to the renderer
		void setBlobPos(float x, float y)
		{
			m_render.setCirclePos(x, y);
		}

		//! return number of projector
		const int projCnt()
		{
			return m_num_proj;
		}

		//! send camera calib result to the renderer: for evaluation
		void setCameraIntrinsic(const cv::Mat_<float>& KK, const cv::Mat_<float>& dist_coeff)
		{
			glm::vec2 fc = glm::vec2((float)KK[0][0], (float)KK[1][1]);
			glm::vec2 cc = glm::vec2((float)KK[0][2], (float)KK[1][2]);
			glm::vec3 kc = glm::vec3(dist_coeff[0][0], dist_coeff[0][1], dist_coeff[0][4]);
			glm::vec2 pc = glm::vec2(dist_coeff[0][2], dist_coeff[0][3]);
			m_render.setCameraIntrin(fc, cc, kc, pc);
		}

	private:
		render::CalibRenderGL m_render;

		unsigned int m_width;
		unsigned int m_height;
		unsigned int m_num_proj;

	};
}

#endif
