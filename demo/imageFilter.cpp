#include "pch.h"
#include "image.h"

//添加椒盐噪声
void AddSalt(Mat image, int n)
{
	for (int k = 0;k < n;k++)
	{
		int i = rand() % image.rows;
		int j = rand() % image.cols;
		if (rand() % 2)	// 随机生成，判断是椒噪声还是盐噪声
		{
			if (image.channels() == 1)
				image.at<uchar>(i, j) = 255;
			else
			{
				image.at<Vec3b>(i, j)[0] = 255;
				image.at<Vec3b>(i, j)[1] = 255;
				image.at<Vec3b>(i, j)[2] = 255;
			}
		}
		else
		{
			if (image.channels() == 1)
				image.at<uchar>(i, j) = 0;
			else
			{
				image.at<Vec3b>(i, j)[0] = 0;
				image.at<Vec3b>(i, j)[1] = 0;
				image.at<Vec3b>(i, j)[2] = 0;
			}
		}
	}
}

//生成高斯噪声
double generateGaussianNoise(double mu, double sigma)
{
	//定义小值
	const double epsilon = numeric_limits<double>::min();
	static double z0, z1;
	static bool flag = false;
	flag = !flag;
	//flag为假构造高斯随机变量X
	if (!flag)
		return z1 * sigma + mu;
	double u1, u2;
	//构造随机变量
	do
	{
		u1 = rand() * (1.0 / RAND_MAX);
		u2 = rand() * (1.0 / RAND_MAX);
	} while (u1 <= epsilon);
	//flag为真构造高斯随机变量
	z0 = sqrt(-2.0 * log(u1)) * cos(2 * CV_PI * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(2 * CV_PI * u2);
	return z0 * sigma + mu;
}

//为图像添加高斯噪声
Mat addGaussianNoise(Mat& srcImag)
{
	Mat dstImage = srcImag.clone();
	for (int i = 0; i < dstImage.rows; i++)
	{
		for (int j = 0; j < dstImage.cols; j++)
		{
			//添加高斯噪声
			dstImage.at<Vec3b>(i, j)[0] = saturate_cast<uchar>(dstImage.at<Vec3b>(i, j)[0] + generateGaussianNoise(2, 0.8) * 32);
			dstImage.at<Vec3b>(i, j)[1] = saturate_cast<uchar>(dstImage.at<Vec3b>(i, j)[1] + generateGaussianNoise(2, 0.8) * 32);
			dstImage.at<Vec3b>(i, j)[2] = saturate_cast<uchar>(dstImage.at<Vec3b>(i, j)[2] + generateGaussianNoise(2, 0.8) * 32);
		}
	}
	return dstImage;
}


Filter::Filter(Mat img)
{
	image = img;
}

Mat Filter::ImageBlurFilter()
{
	Mat dst;
#if 0
	blur(image, dst, Size(3, 3));
#else
	Mat kernal;
	//Mat img1;
	kernal = (Mat_<double>(3, 3) << 1 / 9.0, 1 / 9.0, 1 / 9.0, 1 / 9.0, 1 / 9.0, 1 / 9.0, 1 / 9.0, 1 / 9.0, 1 / 9.0);
	filter2D(image, dst, -1, kernal);
	/*kernal = (Mat_<double>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
	image.convertTo(img1, CV_16UC3);
	filter2D(img1, dst, -1, kernal);
	dst.convertTo(dst2, CV_8UC3, 1 / 9.0);*/
#endif
	return dst;
}

Mat Filter::ImageMedianBlurFilter()
{
	Mat dst;
	medianBlur(image, dst, 9);
	return dst;
}

Mat Filter::ImageSideBlurFilter()
{
	Mat dst;
	Mat kernal[8];
	Mat side_dst[8];
	dst = Mat::zeros(image.size(), image.type());
	kernal[0] = (Mat_<double>(3, 3) << 1 / 6.0, 1 / 6.0, 0, 1 / 6.0, 1 / 6.0, 0, 1 / 6.0, 1 / 6.0, 0);
	kernal[1] = (Mat_<double>(3, 3) << 0, 1 / 6.0, 1 / 6.0, 0, 1 / 6.0, 1 / 6.0, 0, 1 / 6.0, 1 / 6.0);
	kernal[2] = (Mat_<double>(3, 3) << 1 / 6.0, 1 / 6.0, 1 / 6.0, 1 / 6.0, 1 / 6.0, 1 / 6.0, 0, 0, 0);
	kernal[3] = (Mat_<double>(3, 3) << 0, 0, 0, 1 / 6.0, 1 / 6.0, 1 / 6.0, 1 / 6.0, 1 / 6.0, 1 / 6.0);
	kernal[4] = (Mat_<double>(3, 3) << 1 / 4.0, 1 / 4.0, 0, 1 / 4.0, 1 / 4.0, 0, 0, 0, 0);
	kernal[5] = (Mat_<double>(3, 3) << 0, 1 / 4.0, 1 / 4.0, 0, 1 / 4.0, 1 / 4.0, 0, 0, 0);
	kernal[6] = (Mat_<double>(3, 3) << 0, 0, 0, 1 / 4.0, 1 / 4.0, 0, 1 / 4.0, 1 / 4.0, 0);
	kernal[7] = (Mat_<double>(3, 3) << 0, 0, 0, 0, 1 / 4.0, 1 / 4.0, 0, 1 / 4.0, 1 / 4.0);
	for (int i = 0;i < 8;i++)
	{
		filter2D(image, side_dst[i], -1, kernal[i]);
	}
	for (int i = 0;i < image.rows - 1;i++)
	{
		for (int j = 0;j < image.cols - 1;j++)
		{
			int min_side[3];
			int min_num[3] = { 0 };
			min_side[0] = abs(side_dst[0].at<Vec3b>(i, j)[0] - image.at<Vec3b>(i, j)[0]);
			min_side[1] = abs(side_dst[0].at<Vec3b>(i, j)[1] - image.at<Vec3b>(i, j)[1]);
			min_side[2] = abs(side_dst[0].at<Vec3b>(i, j)[2] - image.at<Vec3b>(i, j)[2]);
			for (int k = 1;k < 8;k++)
			{
				if (min(min_side[0], abs(side_dst[k].at<Vec3b>(i, j)[0] - image.at<Vec3b>(i, j)[0])) != min_side[0])
				{
					min_num[0] = k;
					min_side[0] = abs(side_dst[k].at<Vec3b>(i, j)[0] - image.at<Vec3b>(i, j)[0]);
					//dst.at<Vec3b>(i, j)[0] = side_dst[k].at<Vec3b>(i, j)[0];
				}
				if (min(min_side[1], abs(side_dst[k].at<Vec3b>(i, j)[1] - image.at<Vec3b>(i, j)[1])) != min_side[1])
				{
					min_num[1] = k;
					min_side[1] = abs(side_dst[k].at<Vec3b>(i, j)[1] - image.at<Vec3b>(i, j)[1]);
				}
				if (min(min_side[2], abs(side_dst[k].at<Vec3b>(i, j)[2] - image.at<Vec3b>(i, j)[2])) != min_side[2])
				{
					min_num[2] = k;
					min_side[2] = abs(side_dst[k].at<Vec3b>(i, j)[2] - image.at<Vec3b>(i, j)[2]);
				}
			}
			dst.at<Vec3b>(i, j)[0] = side_dst[min_num[0]].at<Vec3b>(i, j)[0];
			dst.at<Vec3b>(i, j)[1] = side_dst[min_num[1]].at<Vec3b>(i, j)[1];
			dst.at<Vec3b>(i, j)[2] = side_dst[min_num[2]].at<Vec3b>(i, j)[2];
		}
	}
	return dst;
}

Mat Filter::ImageGaussianFilter()
{
	Mat dst;
	Mat kernal;
	kernal = (Mat_<float>(3, 3) << 1 / 16.0, 2 / 16.0, 1 / 16.0, 2 / 16.0, 4 / 16.0, 2 / 16.0, 1 / 16.0, 2 / 16.0, 1 / 16.0);
	filter2D(image, dst, -1, kernal);
	return dst;
}
