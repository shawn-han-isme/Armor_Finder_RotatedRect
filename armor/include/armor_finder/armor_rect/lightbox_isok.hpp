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
    double length_width_rate_thresh_max = 10; //设定bbox的宽长比上阈值
	double length_width_rate_thresh_min = 1.2; //设定bbox的宽长比下阈值
	double thresh_binar = 0.02; //二值化取thresh_binar最亮部分
	int thresh_value = 250; // bboxes_light的色度阈值



    //灯条矩形长宽比length_width_rate定义
    double length_width_rate = rect.size.height > rect.size.width ?
    rect.size.height / rect.size.width :
    rect.size.width / rect.size.height;  

    if(length_width_rate<length_width_rate_thresh_max
    && length_width_rate>length_width_rate_thresh_min
    && hsv_isok(mat_imagepart, thresh_binar, thresh_value))
    {
        return true;
    }
    }



    bool hsv_isok(const cv::Mat& in, double proportion, int thresh_value)
{
	cv::Mat in2;
	int rows = in.rows;
	int cols = in.cols;
	
	CvScalar scalar;

	if(in.isContinuous())
	{
		cols *= rows;
		rows = 1;
	}

	IplImage* ipl_in2 = cvCreateImage(cvSize(rows,cols), IPL_DEPTH_8U, 3);
	*ipl_in2 = IplImage(in2);

	std::vector<int> color_value(rows*cols);
	int pos = 0;
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			scalar = cvGet2D(ipl_in2, i, j);
			color_value[pos++] = (int)scalar.val[2];
		}
	}
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

		if((int)scalar.val[1]>254
			&& (int)scalar.val[1]<30
			&& (int)scalar.val[2]>254)
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