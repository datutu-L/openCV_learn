#include "pch.h"
#include "image.h"

/*
* ***** �ݶ��� *****
* ��ͼ��f���ݷ���ͺ᷽�������������΢��
* x �����һ�׵�����fx`=f(i+1,j)-f(i,j)
* y �����һ�׵�����fy`=f(i,j+1)-f(i.j)
* ��x��y�ֱ�ȡ����ֵ��ӣ����߶�������ֵ��ƽ���ڿ�������ֱ�ӵõ��ݶȣ�����ȡ������ֵ���ֵ���͵õ��ݶȣ�
* grad(i.j)=|fx`|+|fy`|	(��������ֵ�ֱ�ƽ������ӣ��ٿ���)
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

/******* Laplace�� ******
* Laplace����	0	1	0
*				1	-4	1
*				0	1	0
*�񻯺��ͼ�� = ԭͼ�� - Laplace����
*���Ϊ��		0	-1	0
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
#else	// ͨ��openCV�ṩ��fiter2D�������þ������ʵ��
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