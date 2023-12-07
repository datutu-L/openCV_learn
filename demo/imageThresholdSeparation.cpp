#include "pch.h"
#include "image.h"

void FixedSeparation(Mat src, Mat dst, int threshold_value)
{
	if (src.empty())
	{
		return;
	}
	for (int i = 0;i < src.rows;i++)
	{
		for (int j = 0;j < src.cols;j++)
		{
			if (src.at<uchar>(i, j) > threshold_value)
				dst.at<uchar>(i, j) = 255;
			else
				dst.at<uchar>(i, j) = 0;
		}
	}
}

void ThresholdSeparation::FixedThreshold(String filename, int threshold_value)
{
	Mat src, dst;
	src = imread(filename, IMREAD_GRAYSCALE);
	dst = Mat::zeros(src.size(), CV_8UC1);
	FixedSeparation(src, dst, threshold_value);
	/*if (src.empty())
	{
		return;
	}
	dst = Mat::zeros(src.size(), CV_8UC1);
	for (int i = 0;i < src.rows;i++)
	{
		for (int j = 0;j < src.cols;j++)
		{
			if (src.at<uchar>(i, j) > threshold_value)
				dst.at<uchar>(i, j) = 255;
			else
				dst.at<uchar>(i, j) = 0;
		}
	}*/
	imshow("src", src);
	imshow("dst", dst);
}

void ThresholdSeparation::OTSUThreshold(String filename)
{
	Mat src, dst;
	src = imread(filename, IMREAD_GRAYSCALE);
	float hist_num[256];
	HistData(src, hist_num);
	float u = 0, u1, u2;
	float n1, n2;
	float w1, w2;
	int sum = src.rows * src.cols;
	float devi, max_devi = 0;
	int max_t = 0;
	for (int j = 0;j < 156;j++)
	{
		u += j * hist_num[j];
	}
	//_cprintf("u = %f\n", u);
	for (int k = 0;k < 256;k++)
	{
		//_cprintf("\nk = %d\n", k);
		u1 = 0;
		n1 = 0;
		for (int i = 0;i <= k;i++)
		{
			n1 += hist_num[i];
			u1 += hist_num[i] * i * sum;
		}
		n1 = sum * n1;
		u1 = u1 / n1;

		u2 = 0;
		for (int j = k + 1;j < 256;j++)
		{
			u2 += hist_num[j] * j * sum;
		}
		n2 = sum - n1;
		u2 = u2 / n2;

		//_cprintf("u1=%f,u2=%f,n1=%d,n2=%d", u1, u2, n1, n2);

		devi = (pow(u1 - u, 2) * n1 + pow(u2 - u, 2) * n2) / sum;
		//_cprintf("%f\t", devi);
		if (devi > max_devi)
		{
			max_devi = devi;
			max_t = k;
		}
	}
	_cprintf("max_devi = %f\nmax_T = %d\n", max_devi, max_t);
	dst = Mat::zeros(src.size(), CV_8UC1);
	FixedSeparation(src, dst, max_t);
	imshow("src", src);
	imshow("OTSU", dst);
}

Mat ThresholdSeparation::KitlteThreshold(Mat src)
{
	Mat dst;
	//src = imread(filename, IMREAD_GRAYSCALE);
	int grad_num = 0;
	int grad_sum = 0;
	int grad;
	int KT = 0;
	for (int i = 1;i < src.rows - 1;i++)
	{
		for (int j = 1;j < src.cols - 1;j++)
		{
			grad = max(abs(src.at<uchar>(i + 1, j) - src.at<uchar>(i - 1, j)), abs(src.at<uchar>(i, j + 1) - src.at<uchar>(i, j - 1)));
			grad_sum += grad * src.at<uchar>(i, j);
			grad_num += grad;
		}
	}
	KT = grad_sum / grad_num;
	_cprintf("KT = %d\n", KT);
	dst = Mat::zeros(src.size(), CV_8UC1);
	FixedSeparation(src, dst, KT);
	imshow("src", src);
	imshow("Kitlte", dst);
	return dst;
}

