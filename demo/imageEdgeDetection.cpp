#include "pch.h"
#include "image.h"

/******* RobertsËã×Ó *********
* Sx =	1	0		Sy =	0	-1
*		0	-1				1	0
*Sx(i,j)=f(i,j)-f(i+1,j+1)	Sy(i,j)=f(i+1,j)-f(i,j+1)
****/
void RobertsEdgeDetection(String filename)
{
	Mat src, roberts, dst;
	src = imread(filename, IMREAD_GRAYSCALE);
	roberts = Mat::zeros(src.size(), src.type());
	dst = Mat::zeros(src.size(), src.type());
#if 1
	for (int i = 0;i < src.rows - 1;i++)
	{
		for (int j = 0;j < src.cols - 1;j++)
		{
			roberts.at<uchar>(i, j) = saturate_cast<uchar>(abs(src.at<uchar>(i, j) - src.at<uchar>(i + 1, j + 1)) +
				abs(src.at<uchar>(i + 1, j) - src.at<uchar>(i, j + 1)));
		}
	}
#else
	Mat kernalx, kernaly;
	Mat Sx, Sy;
	Mat absx, absy;
	kernalx = (Mat_<char>(3, 3) << 0, 0, 0, 0, 1, 0, 0, 0, -1);
	kernaly = (Mat_<char>(3, 3) << 0, 0, 0, 0, 0, -1, 0, 1, 0);
	Sx = Mat::zeros(src.size(), CV_8SC1);
	Sy = Mat::zeros(src.size(), CV_8SC1);
	absx = Mat::zeros(src.size(), CV_8UC1);
	absy = Mat::zeros(src.size(), CV_8UC1);
	filter2D(src, Sx, -1, kernalx);
	filter2D(src, Sy, -1, kernaly);
	convertScaleAbs(Sx, absx);
	convertScaleAbs(Sy, absy);
	addWeighted(Sx, 0.5, Sy, 0.5, 0.0, roberts);
#endif
	imshow("src", src);
	imshow("Roberts", roberts);
}

/******* PrewittËã×Ó *********
* Sx =	-1	0	1		Sy =	1	1	1
*		-1	0	1				0	0	0
*		-1	0	1				-1	-1	-1
*
****/
void PrewittEdgeDetection(String filename)
{
	Mat src, prewitt, dst;
	Mat Sx, Sy;
	src = imread(filename, IMREAD_GRAYSCALE);
	prewitt = Mat::zeros(src.size(), src.type());
	dst = Mat::zeros(src.size(), src.type());
	prewitt.copyTo(Sx);
	prewitt.copyTo(Sy);
	for (int i = 1;i < src.rows - 1;i++)
	{
		for (int j = 1;j < src.cols - 1;j++)
		{
			Sx.at<uchar>(i, j) = saturate_cast<uchar>(abs(src.at<uchar>(i - 1, j + 1) + src.at<uchar>(i, j + 1) +
				src.at<uchar>(i + 1, j + 1) - src.at<uchar>(i - 1, j - 1) - src.at<uchar>(i, j - 1) -
				src.at<uchar>(i + 1, j - 1)));
			Sy.at<uchar>(i, j) = saturate_cast<uchar>(abs(src.at<uchar>(i - 1, j - 1) + src.at<uchar>(i - 1, j) +
				src.at<uchar>(i - 1, j + 1) - src.at<uchar>(i + 1, j - 1) - src.at<uchar>(i + 1, j) -
				src.at<uchar>(i + 1, j + 1)));
		}
	}
	addWeighted(Sx, 0.5, Sy, 0.5, 0.0, prewitt);
	imshow("src", src);
	imshow("Prewitt_x", Sx);
	imshow("Prewitt_y", Sy);
	imshow("Prewitt_x+y", prewitt);
}

/******* SobelËã×Ó *********
* Sx =	-1	0	1		Sy =	1	2	1
*		-2	0	2				0	0	0
*		-1	0	1				-1	-2	-1
*
****/
Mat SobelEdgeDetection(Mat src)
{
	Mat sobel, dst;
	Mat Sx, Sy;
	//src = imread(filename, IMREAD_GRAYSCALE);
	sobel = Mat::zeros(src.size(), src.type());
	dst = Mat::zeros(src.size(), src.type());
	sobel.copyTo(Sx);
	sobel.copyTo(Sy);
	for (int i = 1;i < src.rows - 1;i++)
	{
		for (int j = 1;j < src.cols - 1;j++)
		{
			Sy.at<uchar>(i, j) = saturate_cast<uchar>(abs(src.at<uchar>(i - 1, j + 1) + 2 * src.at<uchar>(i, j + 1) +
				src.at<uchar>(i + 1, j + 1) - src.at<uchar>(i - 1, j - 1) - 2 * src.at<uchar>(i, j - 1) -
				src.at<uchar>(i + 1, j - 1)));
			Sx.at<uchar>(i, j) = saturate_cast<uchar>(abs(src.at<uchar>(i - 1, j - 1) + 2 * src.at<uchar>(i - 1, j) +
				src.at<uchar>(i - 1, j + 1) - src.at<uchar>(i + 1, j - 1) - 2 * src.at<uchar>(i + 1, j) -
				src.at<uchar>(i + 1, j + 1)));
		}
	}
	addWeighted(Sx, 0.5, Sy, 0.5, 0.0, sobel);
	imshow("src", src);
	imshow("Sobel_x", Sx);
	imshow("Sobel_y", Sy);
	return Sx;
	//imshow("Sobel_x+y", sobel);
}

/******* LaplaceËã×Ó *********
*		0	-1	0
*		-1	4	-1
*		0	-1	0
*
****/
void LaplaceEdgeDetection(String filename)
{
	Mat src, laplace, dst;
	src = imread(filename, IMREAD_GRAYSCALE);
	laplace = Mat::zeros(src.size(), src.type());
	dst = Mat::zeros(src.size(), src.type());
	for (int i = 1;i < src.rows - 1;i++)
	{
		for (int j = 1;j < src.cols - 1;j++)
		{
			laplace.at<uchar>(i, j) = saturate_cast<uchar>(4 * src.at<uchar>(i, j) - src.at<uchar>(i - 1, j)
				- src.at<uchar>(i, j - 1) - src.at<uchar>(i, j + 1) - src.at<uchar>(i + 1, j));
		}
	}
	imshow("src", src);
	imshow("Laplace", laplace);
}

/******* Canny±ßÔµ¼ì²â **********/
void CannyEdgeDetection(String filename)
{
	Mat src, canny, dst;
	double shold1, shold2;
	shold1 = 70;
	shold2 = 2 * shold1;
	src = imread(filename, IMREAD_GRAYSCALE);
	canny = Mat::zeros(src.size(), src.type());
	Canny(src, canny, shold1, shold2);
	imshow("src", src);
	imshow("Canny", canny);
}