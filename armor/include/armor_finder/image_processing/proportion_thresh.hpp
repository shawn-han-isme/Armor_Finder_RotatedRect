#pragma once

// #ifdef USE_NEW_CODE
// #undef USE_NEW_CODE
// #endif

// #ifdef DEBUG
// #undef DEBUG
// #endif




// 开始define

// #define DEBUG //在程序中用 #ifdef DEBUG 与 #endif 将debug代码块框起来,实现debug输出 
// #define USE_NEW_CODE //如果保留这一行,则使用新代码; 如果注释掉这一行,则使用旧代码

# include <opencv2/opencv.hpp>
# include <vector>
# include <algorithm>

#ifdef USE_NEW_CODE
namespace sp
{
    void proportion_thresh(const cv::Mat&, const cv::Mat&, double, double);
    int get_threshold_(cv::Mat&, double);
    
    void proportion_thresh(cv::Mat& in, cv::Mat& out, double max_val, double proportion) //han's proportion_thresh
    {
        int rows = in.rows;
        int cols = in.cols;
        
        if(in.isContinuous())
        {
            cols *= rows;
            rows = 1;
        }
        std::vector<uchar> color_value(rows*cols);
        int pos = 0;
        for(int i=0;i<rows;i++)
        {
            //获取第i行首像素指针
            uchar* p = in.ptr<uchar>(i);
            //对第i行的每个像素（Byte）进行操作
            for(int j=0;j<cols;j++)
            {
                color_value[pos++] = p[j];
            }
        }
        std::nth_element(color_value.begin(), color_value.end()-rows*cols*proportion, color_value.end());
        auto thre_iterator = color_value.end()-rows*cols*proportion;
        uchar threshold = *thre_iterator;

        int threshold_int = (int)threshold;

        #ifdef DEBUG
        std::cout << "han's threshold=" << threshold_int << std::endl; //打印计算得出的threshold
        #endif

        cv::threshold(in, out, threshold_int, 255, CV_THRESH_BINARY);
    }

    inline int get_threshold_(cv::Mat& mat, double thre_proportion) //获得模板图像像素值二值化需要的阈值 //han's proportion_thresh
    {
        uint32_t iter_rows = mat.rows;
        uint32_t iter_cols = mat.cols;
        auto sum_pixel = iter_rows * iter_cols;
        if(mat.isContinuous())
        {
            iter_cols = sum_pixel;
            iter_rows = 1;
        }
        int histogram[256];
        memset(histogram, 0, sizeof(histogram));
        for (uint32_t i = 0; i < iter_rows; ++i)
        {
            const uchar* lhs = mat.ptr<uchar>(i);
            for (uint32_t j = 0; j < iter_cols; ++j)
                ++histogram[*lhs++];
        }
        auto left = thre_proportion * sum_pixel;
        int i = 255;
        while((left -= histogram[i--]) > 0);
        return std::max(i, 0);
    }

}

#else

#endif
