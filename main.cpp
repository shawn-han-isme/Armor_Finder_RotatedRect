#ifdef USE_NEW_CODE
#undef USE_NEW_CODE
#endif

#ifdef DEBUG
#undef DEBUG
#endif

#define USE_VIDEO
// #define USE_CAMERA

#define USE_RED
// #define USE_BLUE

// #define USE_HSV_FILTER
#define USE_RGB_FILTER

#define PRINT_CLASSIFIER_RUNTIME

// #define DEBUG
// #define DEBUG_FINDARMORBOX
// #define DEBUG_CLASSIFIER
// #define DEBUG_CLASSIFIER_ORB
// #define DEBUG_PRINT_ARMORNUM

#define USE_NEW_CODE

// #define SHOW_DEBUG_HSV
// #define SHOW_MEDIANBLUR //显示中值滤波图像
// #define SHOW_MONO_COLOR //显示RGB通道分离结果
// #define SHOW_MONO_COLOR_AFTER
// #define SHOW_CONTOURS

// #define SHOW_IMAGEPART_LIGHT //显示灯条矩形截图
// #define SHOW_ARMOR_IMAGE //显示装甲板矩形截图

// #define SHOW_LIGHT //显示灯条矩形
// #define SHOW_ARMOR_UP_RIGHT //显示灯条的匹配矩形（窄长矩形）
#define SHOW_ARMOR //显示完整装甲板矩形（宽矩形）
// #define SHOW_ARMOR_WHOLE //显示完整装甲板矩形（宽矩形）（废）
// #define SHOW_DISTANCE //显示距离

#define FRAME_BY_FRAME
// #define CALSSIFIER_IMAGE_BY_IMAGE
// #define SHOW_CLASSIFIER_IMAGE
// #define CLASSIFIER_OUTPUT //输出分类器结果到"Video/image/dst/negative/和positive"

#define CLASSIFIER_IMAGEPART_ROWS 100
#define CLASSIFIER_IMAGEPART_COLS 120




# include <iostream>
# include <opencv2/opencv.hpp>
# include "other/include/timer.hpp"
# include "other/include/drawText.hpp"
# include "armor/include/show_images/show_images.hpp"
# include "armor/include/armor_finder/armor_rect/find_armor.hpp"
# include "armor/include/armor_finder/image_processing/color_filter.hpp"
// # include "armor/include/armor_finder/distance/PNP.hpp"
// # include "armor/include/armor_finder/image_processing/C-color.hpp"
// # include "armor/include/armor_finder/distance/distance.hpp"

# ifdef USE_NEW_CODE //新代码在下面

int main()
{
    sp::timer timer; //建立计时器

    cv::VideoCapture capture;

    #ifdef USE_CAMERA //使用摄像头
    capture.open("/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0",CV_CAP_V4L);
    
    #ifdef USE_RED
    sp::capture_set(capture,  640,//WIDTH
                              480,//HEIGHT
                              20,//FPS
                              64,//BRIGHTNESS,
                              64,//CONTRAST, 
                              128,//SATURATION
                              40,//HUE, const int 
                              0.003//EXPOSURE
                    );
    #endif

    #ifdef USE_BLUE
    sp::capture_set(capture,  640,//WIDTH
                              480,//HEIGHT
                              20,//FPS
                              64,//BRIGHTNESS,
                              64,//CONTRAST, 
                              128,//SATURATION
                              40,//HUE, const int 
                              0.02//EXPOSURE
                    );
    #endif
    //capture.open(1)

    cv::Mat src;
    cv::Mat src_real;
    capture >> src_real; 
    #endif


    #ifdef USE_VIDEO //使用录像
    // capture.open("../Video/Webcam/全国赛part3.avi");
    // capture.open("../Video/Webcam/全国赛part3_片段.mp4");
    // capture.open("../Video/Webcam/2019-11-12-232726.webm");
    // capture.open("../Video/Webcam/2019-11-12-232804.webm");
    capture.open("../Video/Webcam/2019-11-12-232829.webm");
    // capture.open("../Video/Webcam/2019-10-28-222635.webm");
    // capture.open("../Video/Webcam/2019-10-28-223802.webm");
    // capture.open("../Video/Webcam/2019-10-28-223826.webm");
    // capture.open("../Video/Webcam/2019-10-28-223848.webm");
    cv::Mat src;
    cv::Mat src_real;

    capture >> src_real; 

    #endif

    if(capture.isOpened())
    {
        for(;;) //读取视频循环
        {
            timer.reset(); // 开始计时

            capture >> src_real;

            #ifdef USE_VIDEO
            cv::resize(src_real,src_real,cv::Size(640,480),(0,0), (0,0), CV_INTER_AREA);
            #endif





            // cv::medianBlur(src_real, src, 5); //中值滤波

            // #ifdef USE_RGB_FILTER
            // sp::rgbColorFilter(src, src);
            // #endif

            // #ifdef USE_HSV_FILTER
            // sp::hsvColorFilter(src, src);
            // #endif

            #ifdef USE_RGB_FILTER
            sp::rgbColorFilter(src_real, src);
            #endif

            #ifdef USE_HSV_FILTER
            sp::hsvColorFilter(src_real, src);
            #endif


            #ifdef SHOW_MONO_COLOR
            cv::imshow("SHOW_MONO_COLOR_BEFORE", src);
            #endif
            if(src.empty())
                break;




            //寻找装甲板
            sp::findArmor(src, src_real); 
            

            #ifndef CALSSIFIER_IMAGE_BY_IMAGE
            cv::imshow("Armor_Target", src_real);
            #endif

            #ifdef SHOW_MONO_COLOR_AFTER
            cv::imshow("SHOW_MONO_COLOR_AFTER", src);
            #endif

            std::cout << "程序运行时间：" << timer.get() << "ms" << std::endl; //结束计时

            std::cout<<std::endl;

            #ifdef FRAME_BY_FRAME
            cv::waitKey(0); //逐帧播放
            
            #else
            if(cv::waitKey(10) >= 10)
                break;
            #endif
        }
    }


    
    else
    {
        std::cout << "No capture" << std::endl;
        src = cv::Mat::zeros(480,640,CV_8UC1);
        cv::imshow("image", src);
        cv::waitKey(0);

    }

    return 0;
}


#else

#endif
