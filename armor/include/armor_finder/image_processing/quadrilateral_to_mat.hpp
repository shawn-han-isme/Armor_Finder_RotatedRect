#pragma once

# include <opencv2/opencv.hpp>

namespace sp
{
cv::Mat quadrilateralToMat(cv::Mat& mat_real, cv::Point2f vertices_armor[4])
{
    cv::Mat mat_imagepart;
    cv::Point2f src_points[4], dst_points[4];
    dst_points[1] = cv::Point2f(0, 0);
    dst_points[2] = cv::Point2f(CLASSIFIER_IMAGEPART_COLS, 0);
    dst_points[3] = cv::Point2f(CLASSIFIER_IMAGEPART_COLS, CLASSIFIER_IMAGEPART_ROWS);
    dst_points[0] = cv::Point2f(0, CLASSIFIER_IMAGEPART_ROWS);

    for(int i=0;i<4;i++)
    {
        src_points[i] = vertices_armor[i];
    }

    cv::Mat transMat = getPerspectiveTransform(src_points, dst_points); //得到变换矩阵
    cv::warpPerspective(mat_real, mat_imagepart, transMat, {CLASSIFIER_IMAGEPART_COLS,CLASSIFIER_IMAGEPART_ROWS}); //进行坐标变换

	return mat_imagepart;
}
}
