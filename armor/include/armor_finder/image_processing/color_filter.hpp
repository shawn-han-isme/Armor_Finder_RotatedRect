#pragma once

# include <opencv2/opencv.hpp>

namespace sp
{

void rgbColorFilter(cv::Mat& src_real, cv::Mat& src)
{
	std::vector<cv::Mat> channels;//定义Mat类型的向量
	cv::split(src_real, channels);//通道分离
	cv::Mat blue = channels.at(0);
	cv::Mat green = channels.at(1);
	cv::Mat red = channels.at(2);

	#ifdef USE_RED
	src = red;
	#endif

	#ifdef USE_BLUE
	src = blue;
	#endif

}


void hsvColorFilter(cv::Mat& inputImage, cv::Mat& outputImage)
{
	int i, j;

	int rows = inputImage.rows;
	int cols = inputImage.cols;

	cv::Mat hsv_mat;
	
	if(inputImage.isContinuous())
	{
		cols *= rows;
		rows = 1;
	}

	
	
    cv::cvtColor(inputImage, hsv_mat, CV_RGB2HSV);

	// IplImage* image = cvCreateImage(cvSize(rows,cols), IPL_DEPTH_8U, 3);
	// *image = IplImage(inputImage);
    IplImage* hsv = cvCreateImage(cvSize(rows,cols), IPL_DEPTH_8U, 3);  
	*hsv = IplImage(hsv_mat);

	int width = hsv->width;
	int height = hsv->height;
	
	#ifdef USE_RED
	for (i = 0; i < height; i++)
		for (j = 0; j < width; j++)
		{
			CvScalar s_hsv = cvGet2D(hsv, i, j);//获取像素点为（j, i）点的HSV的值 
			
			CvScalar s;

			if (( //红色的HSV定义
                // s_hsv.val[0]>90	&&
                // s_hsv.val[0]<130 &&
				((s_hsv.val[2]>130))
                ))
			// if(false)
			{
				#ifdef DEBUG
				std::cout <<"HSV: "<<"("<<s.val[0]<<","<<s.val[1]<<","<<s.val[2]<<")"<<std::endl;
				#endif

				s.val[0]=255;
				s.val[1]=255;
				s.val[2]=255;
				cvSet2D(hsv, i ,j, s);

				
			}
			else //将非红色赋值为黑色
			{
				s.val[0]=0;
				s.val[1]=0;
				s.val[2]=0;
				cvSet2D(hsv, i ,j, s);
			}
			
		}
	#endif

	#ifdef USE_BLUE
	for (i = 0; i < height; i++)
		for (j = 0; j < width; j++)
		{
			CvScalar s_hsv = cvGet2D(hsv, i, j);//获取像素点为（j, i）点的HSV的值 
			
			CvScalar s;

			if (( //蓝色的HSV定义
                // s_hsv.val[0]>90	&&
                // s_hsv.val[0]<130 &&
				((s_hsv.val[2]>254))
                ))
			// if(false)
			{
				#ifdef DEBUG
				std::cout <<"HSV: "<<"("<<s.val[0]<<","<<s.val[1]<<","<<s.val[2]<<")"<<std::endl;
				#endif

				s.val[0]=255;
				s.val[1]=255;
				s.val[2]=255;
				cvSet2D(hsv, i ,j, s);

				
			}
			else //将非蓝色赋值为黑色
			{
				s.val[0]=0;
				s.val[1]=0;
				s.val[2]=0;
				cvSet2D(hsv, i ,j, s);
			}
		}
	#endif


    outputImage=cv::cvarrToMat(hsv);

	cv::cvtColor(outputImage, outputImage, CV_RGB2GRAY);

    #ifdef SHOW_DEBUG_HSV
	cvNamedWindow("hsv_filter");
	cvShowImage("hsv_filter", hsv);
    #endif

	// cvReleaseImage(&hsv);

}
}
