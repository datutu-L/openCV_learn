#ifndef __IMAGE_H
#define __IMAGE_H

#include "pch.h"
#include <conio.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

void ImageShow();
Mat ImageGray(String filename);
void HistData(Mat img, float* hist_num);	// 计算各像素点的概率
void DrawHist(String filename);
void HistEqualize(String filename);

//图像锐化
void ImageGradSharpen(String filename);
void ImageLaplaceSharpen(String filename);

//边缘检测
void RobertsEdgeDetection(String filename);
void PrewittEdgeDetection(String filename);
Mat SobelEdgeDetection(Mat src);
void LaplaceEdgeDetection(String filename);
void CannyEdgeDetection(String filename);

//滤波
void AddSalt(Mat image, int n);
Mat addGaussianNoise(Mat& srcImag);

//图像校正
void ImageWarpAffine(String filename);
void ImageWrapPerspective(Mat src, Point2f* srcPoint, Point2f* dstPoint, Size dst_size);

//相机标定
void SignalCamera();
void DoubleCamera();
void StereoMatch();

//图像滤波
class Filter
{
public:
	Mat image;
	Filter(Mat img);
	Mat ImageBlurFilter();
	Mat ImageMedianBlurFilter();
	Mat ImageSideBlurFilter();
	Mat ImageGaussianFilter();
};

//阈值分隔
//void FixedThreshold(String filename, int threshold_value);
class ThresholdSeparation
{
public:
	void FixedThreshold(String filename, int threshold_value);
	void OTSUThreshold(String filename);
	Mat KitlteThreshold(Mat src);
};

//车道线检测
//void DrivewaySplitAndMix(String filename);
void DrivewaySplitAndMix(Mat src_BGR);
//void Process2(String filename1, String filename2);
void Process2(Mat src2, Mat src);
void HoughLineDetection(String filename);
void CarLineVideo(String filename);
Mat CarLineDetection(Mat src_BGR); //合并 DrivewaySplitAndMix 和 Process2

//瑕疵检测
void Brisk_BlemishDetection(String filename1, String filename2);
void ORB_BlemishDetection(String fiename, String filename);

//目标检测
void TargetTracking();

#endif // !__IMAGE_H

