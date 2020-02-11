#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>


#ifdef SHOW_DISTANCE
#include "../distance/PNP.hpp"
#endif

#define USE_NEW_CODE


#ifdef USE_NEW_CODE //新代码在下面

namespace sp
{
    bool lightbox_isok(cv::Mat& mat_imagepart, cv::RotatedRect rect);
    bool hsv_isok(const cv::Mat& in, double proportion, int thresh_value);


    bool lightbox_isok(cv::Mat& mat_imagepart, cv::RotatedRect rect)
    {
		# ifdef DEBUG
        std::cout << "开始判断lightbox" << std::endl;
        std::cout << std::endl;
        # endif

		double length_width_rate_thresh_max = 20; //设定bbox的长宽比上阈值
		double length_width_rate_thresh_min = 2.7; //设定bbox的长宽比下阈值
		double thresh_binar = 0.02; //二值化取thresh_binar最亮部分
		int thresh_value = 220; // bboxes_light的色度阈值

		//灯条矩形长宽比length_width_rate定义
		double length_width_rate = rect.size.height > rect.size.width ?
		rect.size.height / rect.size.width :
		rect.size.width / rect.size.height;  

		# ifdef DEBUG
        std::cout << "设定阈值完成，定义长宽比完成" << std::endl;
        std::cout << std::endl;
        # endif

		if(length_width_rate<length_width_rate_thresh_max
		&& length_width_rate>length_width_rate_thresh_min
		// && hsv_isok(mat_imagepart, thresh_binar, thresh_value)
		)
		{
			return true;
		}
    }



    bool hsv_isok(const cv::Mat& in, double proportion, int thresh_value)
{
	# ifdef DEBUG
	std::cout << "开始通过hsv判断lightbox" << std::endl;
	std::cout << std::endl;
	# endif

	cv::Mat in2;
	int rows = in.rows;
	int cols = in.cols;

	cv::cvtColor(in, in2, CV_RGB2HSV);
	CvScalar scalar;

	if(in.isContinuous())
	{
		cols *= rows;
		rows = 1;
	}

	# ifdef DEBUG
	std::cout << "已获取image_part的rows: " << rows << "和cols: " << cols << std::endl;
	std::cout << std::endl;
	# endif

	IplImage* ipl_in2 = cvCreateImage(cvSize(rows,cols), IPL_DEPTH_8U, 3);
	*ipl_in2 = IplImage(in2);

	# ifdef DEBUG
	std::cout << "已获取IplImage" << std::endl;
	std::cout << std::endl;
	# endif

	std::vector<int> color_value(rows*cols);
	int pos = 0;

	# ifdef DEBUG
	std::cout << "已设定color_value容器，开始遍历hsv像素" << std::endl;
	std::cout << std::endl;
	# endif

	for(int i=0;i<rows;i++)
	{
		# ifdef DEBUG
		std::cout << "开始遍历i：" << i << std::endl;
		std::cout << std::endl;
		# endif

		for(int j=0;j<cols;j++)
		{
			# ifdef SHOW_DEBUG_HSV
			std::cout << "开始遍历j：" << j << std::endl;
			std::cout << std::endl;
			# endif

			scalar = cvGet2D(ipl_in2, i, j);

			# ifdef SHOW_DEBUG_HSV
			std::cout << "已获取(" << i << "," << j << ")点的像素值" << std::endl;
			std::cout << std::endl;
			# endif

			color_value[pos++] = (int)scalar.val[2];

			# ifdef SHOW_DEBUG_HSV
			std::cout << "hsv像素值：" << color_value[pos-1] << std::endl;
			std::cout << std::endl;
			# endif
		}
	}

	# ifdef DEBUG
	std::cout << "遍历hsv像素结束" << std::endl;
	std::cout << std::endl;
	# endif

	std::nth_element(color_value.begin(), color_value.end()-rows*cols*proportion, color_value.end());
	auto thre_iterator = color_value.end()-rows*cols*proportion;
	int threshold_int = *thre_iterator;

	#ifdef DEBUG
	std::cout << "threshold=" << threshold_int << std::endl; //打印计算得出的threshold
	#endif

	if(threshold_int>=thresh_value) // 筛亮度
	{
		// hsv筛颜色

		// 获得bboxes_light的中点坐标和中点hsv亮度
		scalar = cvGet2D(ipl_in2, rows/2, cols/2);

		if((int)scalar.val[2]>240
			// && (int)scalar.val[1]>254
			// && (int)scalar.val[1]<30
			)
		{
			#ifdef DEBUG
			std::cout << "饱和度" << (int)scalar.val[1] << std::endl;
			std::cout << "明度值" << (int)scalar.val[2] << std::endl;
			#endif

			return true;
		}
		else
		{
			#ifdef DEBUG
			std::cout << "未获取HSV" << std::endl;
			#endif
		}
		
		#ifdef DEBUG
		std::cout << "true" << std::endl;
		#endif
	}
	else 
	{
		return false;
		#ifdef DEBUG
		std::cout << "false" << std::endl;
		#endif
	}
	
}


}




#else //旧代码在下面

#endif
