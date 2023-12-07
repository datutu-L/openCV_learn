#include "pch.h"
#include "image.h"

/*
* ***** 梯度锐化 *****
* 对图像f在纵方向和横方向两个方向进行微分
* x 方向的一阶导数：fx`=f(i+1,j)-f(i,j)
* y 方向的一阶导数：fy`=f(i,j+1)-f(i.j)
* 将x和y分别取绝对值相加（或者对这两个值，平方在开方），直接得到梯度；或者取这两个值最大值，就得到梯度；
* grad(i.j)=|fx`|+|fy`|	(或者两个值分别平方后相加，再开方)
* grag(i,j)=max(|fx`|, |fy`|)
*/
void ImageGradSharpen(String filename)	
{
	Mat src, gray, grad, dst;
	src = imread(filename, IMREAD_COLOR);
	cvtColor(src, gray, COLOR_BGR2GRAY);
	if (src.empty())
	{
		return;
	}
	grad = Mat::zeros(gray.size(), gray.type());
	dst = Mat::zeros(gray.size(), gray.type());
	for (int i = 0;i < gray.rows - 1;i++)
	{
		for (int j = 0;j < gray.cols - 1;j++)
		{
			grad.at<uchar>(i, j) = saturate_cast<uchar>(abs(gray.at<uchar>(i + 1, j)
				- gray.at<uchar>(i, j)) + abs(gray.at<uchar>(i, j + 1) - gray.at<uchar>(i, j)));
			dst.at<uchar>(i, j) = saturate_cast<uchar>(gray.at<uchar>(i, j) - grad.at<uchar>(i, j));
		}
	}
	namedWindow("srcImage", WINDOW_AUTOSIZE);
	namedWindow("grad", WINDOW_AUTOSIZE);
	namedWindow("dstImage", WINDOW_AUTOSIZE);
	imshow("srcImage", gray);
	imshow("grad", grad);
	imshow("dstImage", dst);
}

/******* Laplace锐化 ******
* Laplace算子	0	1	0
*				1	-4	1
*				0	1	0
*锐化后的图像 = 原图像 - Laplace算子
*结果为：		0	-1	0
*				-1	5	-1
*				0	-1	0
*/
void ImageLaplaceSharpen(String filename)
{
	Mat src, gray, laplace, dst;
	src = imread(filename, IMREAD_COLOR);
	if (src.empty())
	{
		return;
	}
	cvtColor(src, gray, COLOR_BGR2GRAY);
	laplace = Mat::zeros(gray.size(), gray.type());
	dst = Mat::zeros(gray.size(), gray.type());
#if 1
	for (int i = 1;i < gray.rows - 1;i++)
	{
		for (int j = 1;j < gray.cols - 1;j++)
		{
			laplace.at<uchar>(i, j) = saturate_cast<uchar>(gray.at<uchar>(i - 1, j) + gray.at<uchar>(i, j - 1)
				+ gray.at<uchar>(i, j + 1) + gray.at<uchar>(i + 1, j) - 4 * gray.at<uchar>(i, j));
			dst.at<uchar>(i, j) = saturate_cast<uchar>(gray.at<uchar>(i, j) - laplace.at<uchar>(i, j));
		}
	}
#else	// 通过openCV提供的fiter2D函数利用矩阵卷积实现
	Mat kernal_laplace, kernal_dst;
	kernal_laplace = (Mat_<char>(3, 3) << 0, 1, 0, 1, -4, 1, 0, 1, 0);
	kernal_dst = (Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	filter2D(gray, laplace, -1, kernal_laplace);
	filter2D(gray, dst, -1, kernal_dst);
#endif
	imshow("srcImage", gray);
	imshow("laplace", laplace);
	imshow("dstImage", dst);
}