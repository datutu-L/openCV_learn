#include "pch.h"
#include "image.h"

using namespace cv;
using namespace std;

void HistData(Mat img, float* hist_num);
void DrawLine(String window_name, float* num);

Mat ImageGray(String filename)
{
	Mat colorImg, grayImg;
	colorImg = imread(filename, IMREAD_COLOR);
	if (colorImg.empty())
	{
		return colorImg;
	}
#if 0

	// ʹ��at������ȡ�����þ���Mat�е�Ԫ������
	int r = colorImg.rows;
	int c = colorImg.cols;
	grayImg.create(colorImg.rows, colorImg.cols, CV_8UC1);
	for (int i = 0;i < r;i++)
	{
		for (int j = 0;j < c;j++)
		{
			//Mat�е�.at����
			//grayImg.at<char>(i, j)�����ûҶ�ͼ��i��j�еĵ�
			//colorImg.at<Vec3b>(i,j)[k]��ȡ����ɫͼ����i��j���е�kͨ������ɫ�㡣����Vec3b��ͼ������ֵ�����͡�
			grayImg.at<char>(i, j) = 0.11 * colorImg.at<Vec3b>(i, j)[0]
				+ 0.59 * colorImg.at<Vec3b>(i, j)[1]
				+ 0.3 * colorImg.at<Vec3b>(i, j)[2];
		}
	}

#else

	// ʹ��ָ��ptr����ȡ�����þ���Mat�е�Ԫ������
	grayImg.create(colorImg.rows, colorImg.cols, CV_8UC1);
	int r = colorImg.rows;
	int c = colorImg.cols;
	int channel = colorImg.channels();
	for (int i = 0;i < r;i++)
	{
		uchar* p = colorImg.ptr<uchar>(i);
		for (int j = 0;j < c * channel;j = j + channel)
		{
			grayImg.ptr<uchar>(i)[j / 3] = (uchar)(0.11 * p[j] + 0.59 * p[j + 1] + 0.3 * p[j + 2]);
		}
	}

#endif // 1


	namedWindow("colorImage", WINDOW_AUTOSIZE);
	namedWindow("grayImage", WINDOW_AUTOSIZE);
	imshow("colorImage", colorImg);
	imshow("grayImage", grayImg);
	return grayImg;
}

void DrawHist(String filename)	// ���ƻҶ�ֱ��ͼ
{
	Mat grayImg;
	grayImg = ImageGray(filename);
	/*int pix_num[256] = { 0 };*/
	float hist_num[256];
	HistData(grayImg, hist_num);
	DrawLine("histImage", hist_num);
}

void HistData(Mat img, float* hist_num)	// ��������ص�ĸ���
{
	int pix_num[256] = { 0 };
	for (int i = 0;i < img.rows;i++)
	{
		for (int j = 0;j < img.cols;j++)
		{
			pix_num[img.at<uchar>(i, j)]++;
		}
	}
	for (int i = 0;i < 256;i++)
	{
		hist_num[i] = (float)pix_num[i] / (img.rows * img.cols);
	}
}

void DrawLine(String window_name, float *num)	// �����ݻ���ֱ��ͼ
{
	Mat histImg = Mat::zeros(200, 256, CV_8UC3);
	Point p1[256], p2[256];
	for (int i = 0;i < histImg.cols;i++)
	{
		p1[i].x = i;
		p1[i].y = histImg.rows - 1 - (int)(10000 * num[i]);
		p2[i].x = i;
		p2[i].y = histImg.rows - 1;
		line(histImg, p1[i], p2[i], Scalar(0, 255, 0), 2, LINE_8);
	}
	namedWindow(window_name, WINDOW_AUTOSIZE);
	imshow(window_name, histImg);
}

void HistEqualize(String filename)	// �Ҷ�ֱ��ͼ���⻯
{
	Mat grayImg1, grayImg2, grayImg3;	// ԭʼ�Ҷ�ͼ�����⻯��ĻҶ�ͼ
	grayImg1 = ImageGray(filename);
	grayImg2 = Mat::zeros(grayImg1.size(), grayImg1.type());
	float hist1_num[256];	// ԭʼ�ĻҶ�ֱ��ͼ
	float hist2_num[256];	// ���⻯��ĻҶ�ֱ��ͼ
	float hist3_num[256];	// ���⻯��ĻҶ�ֱ��ͼ��ʹ�ÿ⺯����
	uchar equalize_pix[256] = { 0 };
	float p = 0;
	HistData(grayImg1, hist1_num);
	for (int i = 0;i < 256;i++)
	{
		/*for (int j = 0;j <= i;j++)*/
		p += hist1_num[i];
		equalize_pix[i] = (uchar)(255 * p);
	}
	for (int i = 0;i < grayImg1.rows;i++)
	{
		for (int j = 0;j < grayImg1.cols;j++)
		{
			grayImg2.at<uchar>(i, j) = equalize_pix[grayImg1.at<uchar>(i, j)];
		}
	}

	/* ֱ��ʹ��openCV�ṩ�ĺ���ʵ�ֻҶ�ֱ��ͼ���⻯*/
	equalizeHist(grayImg1, grayImg3);	

	HistData(grayImg2, hist2_num);
	HistData(grayImg3, hist3_num);
	imshow("equalizeImage", grayImg2);
	DrawLine("hist1", hist1_num);
	DrawLine("hist2", hist2_num);
	DrawLine("hist3", hist3_num);
}