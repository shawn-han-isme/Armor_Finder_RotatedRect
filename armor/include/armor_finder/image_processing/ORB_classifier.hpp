#pragma once

#include <opencv2/xfeatures2d.hpp>

// #define DRAW_IMAGE_FEATURE_MATCH

namespace sp //使用命名空间sp
{
bool ORB_classifier_isok(const cv::Mat& img2)
{
    #ifdef PRINT_CLASSIFIER_RUNTIME
    sp::timer timer_classifier_orb; //建立计时器
    timer_classifier_orb.reset(); // 开始计时
	#endif

    try
    { 
    //【1】载入原图片并显示+载入的图像是二值化之后的图像
    //img1是模板图像，img2是待检测图
    cv::Mat img1 = cv::imread("../Video/image/src/armor/T4.jpg",0);
    if(!img1.data||!img2.data)//检测图片是否读取成功
    {
        std::cout<<"读取图片错误，请确定目录下是否存在该图片"<<std::endl;
    }
    // cv::medianBlur(img1, img1, 3); //中值滤波
    // cv::medianBlur(img2, img2, 3); //中值滤波

    //【2】定义需要用到的变量和类
    cv::Ptr<cv::ORB> detector = cv::ORB::create(200,1.2); //定义一个ORB特征检测类对象detector
    std::vector<cv::KeyPoint> keypoint_1, keypoint_2;//放置特征点
    cv::Mat descriptors_1, descriptors_2;

    //【3】调用detect函数检测出SURF特征关键点，保存在vector容器中
    detector->detectAndCompute(img1,cv::Mat(),keypoint_1,descriptors_1);
    detector->detectAndCompute(img2,cv::Mat(),keypoint_2,descriptors_2);
    
    #ifdef DEBUG_CLASSIFIER_ORB
    if(!descriptors_1.data)
    {
        std::cout<<"> descriptors_1无内容"<<std::endl;
    }
    if (!descriptors_2.data)
    {
        std::cout<<"> descriptors_2无内容"<<std::endl;
    }
    #endif
    

    
    //【4】基于FLANN的描述符对象匹配
    std::vector<cv::DMatch> matches;
    // 初始化flann匹配
    cv::flann::Index flannIndex(descriptors_1, cv::flann::LshIndexParams(12,20,2), cvflann::FLANN_DIST_HAMMING);

    //【10】匹配和测试描述符，获取两个最邻近的描述符
    cv::Mat matchIndex(descriptors_1.rows, 2, CV_32SC1);
    cv::Mat matchDistance(descriptors_1.rows, 2, CV_32FC1);

    flannIndex.knnSearch(descriptors_2, matchIndex, matchDistance, 2, cv::flann::SearchParams());//调用K邻近算法

    //【11】根据劳氏算法(Low's algorithm)选出优秀的匹配
    std::vector<cv::DMatch> good_matches;
    for(int i=0;i<matchDistance.rows;i++)
    {
        if(matchDistance.at<float>(i,0) < 0.6*matchDistance.at<float>(i,1))
        {
            cv::DMatch dmatches(i, matchIndex.at<int>(i,0), matchDistance.at<float>(i,0));
            good_matches.push_back(dmatches);
        }
    }

    //【12】绘制并显示匹配窗口
    cv::Mat img_matches;
    cv::drawMatches(img2,keypoint_2,img1,keypoint_2,good_matches,img_matches);

    //【13】输出相关匹配点信息
    // for(int i=0;i<good_matches.size();i++)
    // {
    //     std::cout<<"> 符合条件的匹配点 "<<i<<" 特征点1："<<good_matches[i].queryIdx<<" -- 特征点2："<<good_matches[i].trainIdx<<std::endl;
    // }
    
    //【14】打印特征信息
    
    // std::cout<<"> img1检测到特征点"<<keypoint_1.size()<<"个"<<std::endl;
    // std::cout<<"> img2检测到特征点"<<keypoint_2.size()<<"个"<<std::endl;
    #ifdef DEBUG_CLASSIFIER_ORB
    std::cout<<"> 共匹配到特征点"<<good_matches.size()<<"对"<<std::endl;
    #endif

    //【15】绘制特征图像
    #ifdef DRAW_IMAGE_FEATURE_MATCH
    cv::Mat img1_keypoint, img2_keypoint;
    cv::drawKeypoints(img1,keypoint_1,img1_keypoint);
    cv::drawKeypoints(img2,keypoint_2,img2_keypoint);
    cv::imshow("> 特征点检测效果图1",img1_keypoint);
    cv::imshow("> 特征点检测效果图2",img2_keypoint);
    cv::imshow("匹配效果图",img_matches);
    // cv::waitKey(0);
    #endif

    if(good_matches.size()>1)
    {
        #ifdef PRINT_CLASSIFIER_RUNTIME
	    std::cout << "> 二级分类器运行时间：" << timer_classifier_orb.get() << "ms" << std::endl; //结束计时
		#endif

        return true;
    }
    else
    {
        #ifdef PRINT_CLASSIFIER_RUNTIME
	    std::cout << "> 二级分类器运行时间：" << timer_classifier_orb.get() << "ms" << std::endl; //结束计时
		#endif
        
        return false;
    }
    

    }
    catch (std::exception& e) 
    {
        #ifdef PRINT_CLASSIFIER_RUNTIME
	    std::cout << "> 二级分类器运行时间：" << timer_classifier_orb.get() << "ms" << std::endl; //结束计时
		#endif

        #ifdef DEBUG_CLASSIFIER_ORB
        std::cout << "> ORB分类器出错:" << std::endl; 
        // std::cout << "> ORB分类器出错:" << std::endl <<"> Standard exception: " << std::endl << e.what() << std::endl; 
        std::cout << "> ORB返回false" << std::endl;
        #endif

        return false;
    } 
}
}
