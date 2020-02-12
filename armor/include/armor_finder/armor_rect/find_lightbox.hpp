#pragma once

# include <iostream>
# include <opencv2/opencv.hpp>
# include "lightbox_isok.hpp"
# include "../image_processing/rotate_rect_to_mat.hpp"

namespace sp
{
std::vector<cv::RotatedRect> findLightBox(cv::Mat& mat, cv::Mat& mat_real)
//mat是已经经过了HSV通道分离的图像，mat_real是摄像头探测到的原图
{
	std::vector<std::vector<cv::Point>> contours; //轮廓容器
    std::vector<cv::RotatedRect> light_boxes; //灯条矩形容器

    // constexpr float thresh_iou = 0.8; //IOU的阈值
	int thresh_value = 250; // light_boxes的色度阈值

    // 找到轮廓，放入contours
    cv::findContours(mat, 
                    contours, 
                    CV_RETR_LIST, // 检测所有的轮廓，包括内围、外围轮廓，但是检测到的轮廓不建立等级关系
                    CV_CHAIN_APPROX_NONE //仅保存轮廓的拐点信息，把所有轮廓拐点处的点保存入contours向量内，拐点与拐点之间直线段上的信息点不予保留
                    );

    # ifdef DEBUG
    std::cout << "找到" << contours.size() << "个轮廓,开始遍历轮廓" << std::endl;
    std::cout << std::endl;
    # endif

    #ifdef SHOW_CONTOURS
    cv::Mat show_contours = mat_real;
    for(int i=0; i<contours.size(); i++)
    {
        cv::drawContours(show_contours, contours, -1, {0,255,0});
    }
    cv::imshow("show_contours", show_contours);
    #endif



    for(int i=0; i<contours.size(); i++) //遍历轮廓，获取符合要求的最小矩形，并将符合条件的最小矩形放入light_boxes灯条矩形容器
    {
        # ifdef DEBUG
        std::cout << "开始遍历第" << i << "个轮廓" << std::endl;
        std::cout << std::endl;
        # endif

        cv::RotatedRect rect=minAreaRect(contours[i]);
        cv::Point2f vertices[4]; 
        rect.points(vertices); //获取旋转矩形四个顶点
        cv::Point2f center = rect.center;//外接矩形中心点坐标

        if(
            !(0<vertices[0].y && vertices[0].y<mat.rows
            &&0<vertices[0].x && vertices[0].x<mat.cols
            &&0<vertices[1].y && vertices[1].y<mat.rows
            &&0<vertices[1].x && vertices[1].x<mat.cols
            &&0<vertices[2].y && vertices[2].y<mat.rows
            &&0<vertices[2].x && vertices[2].x<mat.cols
            &&0<vertices[3].y && vertices[3].y<mat.rows
            &&0<vertices[3].x && vertices[3].x<mat.cols)
        )
        {
            # ifdef DEBUG
            std::cout << "过滤相同的矩形成功" << std::endl;
            std::cout << std::endl;
            # endif

            continue; //过滤相同的矩形
        }

        
        # ifdef DEBUG
        std::cout << "开始获取image_part" << std::endl;
        std::cout << std::endl;
        # endif
        
        if(!rotateRectToMat_isok(mat_real, rect)) 
        {       
        # ifdef DEBUG
        std::cout << "rotate_rect旋转出界" << std::endl;
        std::cout << std::endl;
        # endif

        continue;
        }

        cv::Mat mat_imagepart = sp::rotateRectToMat(mat_real, rect);

        # ifdef DEBUG
        std::cout << "获取image_part成功" << std::endl;
        std::cout << std::endl;
        # endif

        if(mat_imagepart.empty()) continue;

        if(sp::lightbox_isok(mat_imagepart, rect))
        // if(true)
        {
            # ifdef DEBUG
            std::cout << "判断lightbox结束" << std::endl;
            std::cout << std::endl;
            # endif

            // 画灯条矩形
            #ifdef SHOW_LIGHT
            for (int i = 0; i < 4; i++)//画矩形
            {
                #ifdef SHOW_MONO_COLOR				
                line(mat, vertices[i], vertices[(i + 1) % 4], {255}, 2, 8, 0);
                #endif


                #ifdef USE_RED
                line(mat_real, vertices[i], vertices[(i + 1) % 4], {255, 0, 0}, 2, 8, 0);
                #endif

                #ifdef USE_BLUE
                line(mat_real, vertices[i], vertices[(i + 1) % 4], {0, 0, 255}, 2, 8, 0);                
                #endif
            }
            #endif

            //将灯条放到容器里
            light_boxes.push_back(rect);
            
            # ifdef DEBUG
            std::cout << "本帧筛选出" << light_boxes.size() << "个可能灯条" << std::endl;
            std::cout << std::endl;
            # endif

        }
        else
        {
            # ifdef DEBUG
            std::cout << "未通过lightbox_isok" << std::endl;
            std::cout << std::endl;
            # endif
        }
        
    }
    return light_boxes;
}

   








// 旋转矩形的长宽比
static double lw_rate(const cv::RotatedRect &rect) {
    return rect.size.height > rect.size.width ?
           rect.size.height / rect.size.width :
           rect.size.width / rect.size.height;
}
// 轮廓面积和其最小外接矩形面积之比
static double areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
    return cv::contourArea(contour) / rect.size.area();
}
// 判断轮廓是否为一个灯条
static bool isValidLightBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
    return (1.2 < lw_rate(rect) && lw_rate(rect) < 10) &&
           //           (rect.size.area() < 3000) &&
           ((rect.size.area() < 50 && areaRatio(contour, rect) > 0.4) ||
            (rect.size.area() >= 50 && areaRatio(contour, rect) > 0.6));
}
// 判断两个灯条区域是同一个灯条
static bool isSameBlob(cv::RotatedRect blob1, cv::RotatedRect blob2) {
    auto dist = blob1.center - blob2.center;
    return (dist.x * dist.x + dist.y * dist.y) < 9;
}

}
