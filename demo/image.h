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
void HistData(Mat img, float* hist_num);	// ��������ص�ĸ���
void DrawHist(String filename);
void HistEqualize(String filename);

//ͼ����
void ImageGradSharpen(String filename);
void ImageLaplaceSharpen(String filename);

//��Ե���
void RobertsEdgeDetection(String filename);
void PrewittEdgeDetection(String filename);
Mat SobelEdgeDetection(Mat src);
void LaplaceEdgeDetection(String filename);
void CannyEdgeDetection(String filename);

//�˲�
void AddSalt(Mat image, int n);
Mat addGaussianNoise(Mat& srcImag);

//ͼ��У��
void ImageWarpAffine(String filename);
void ImageWrapPerspective(Mat src, Point2f* srcPoint, Point2f* dstPoint, Size dst_size);

//����궨
void SignalCamera();
void DoubleCamera();
void StereoMatch();

//ͼ���˲�
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

//��ֵ�ָ�
//void FixedThreshold(String filename, int threshold_value);
class ThresholdSeparation
{
public:
	void FixedThreshold(String filename, int threshold_value);
	void OTSUThreshold(String filename);
	Mat KitlteThreshold(Mat src);
};

//�����߼��
//void DrivewaySplitAndMix(String filename);
void DrivewaySplitAndMix(Mat src_BGR);
//void Process2(String filename1, String filename2);
void Process2(Mat src2, Mat src);
void HoughLineDetection(String filename);
void CarLineVideo(String filename);
Mat CarLineDetection(Mat src_BGR); //�ϲ� DrivewaySplitAndMix �� Process2

//覴ü��
void Brisk_BlemishDetection(String filename1, String filename2);
void ORB_BlemishDetection(String fiename, String filename);

//Ŀ����
void TargetTracking();

#endif // !__IMAGE_H

