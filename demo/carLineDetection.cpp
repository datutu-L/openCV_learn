#include "pch.h"
#include "image.h"

void WinSlide(Mat output_image, vector<Point2f>* base_point, int win_num, Scalar markcolor);
//曲线拟合函数
void polyfit(int n, vector <double> x, vector <double> y, int poly_n, double a[]);
void gauss_solve(int n, double A[], double x[], double b[]);

void DrivewaySplitAndMix(Mat src_BGR)
{
	Mat dst, src_HLS, dst_L, dst_S;
	Mat channels[3];
	ThresholdSeparation fixed;
	//src_BGR = imread(filename, IMREAD_COLOR);
	cvtColor(src_BGR, src_HLS, COLOR_BGR2HLS);
	split(src_HLS, channels);
	//dst_L = SobelEdgeDetection(channels[1]);
	Sobel(channels[1], dst_L, -1, 1, 0);
	//dst_S = fixed.KitlteThreshold(channels[2]);
	inRange(channels[2], Scalar(170, 0, 0), Scalar(255, 0, 0), dst_S);
	destroyAllWindows();
	dst = dst_L + dst_S;

	imshow("src", src_BGR);
	imshow("Sobel_H", dst_L);
	imshow("Threshold_S", dst_S);
	imshow("dst", dst);
	imwrite(".\\data\\test_images\\preprocess.jpg", dst);
}

/*针对预处理后的车道线图像，进行透视变换，统计列直方图，提取车道线位置，通过滑窗确定车道线的像素值，
* 并进行多项式拟合，绘制出车道区域并逆投影到原始图像中。
*/
void Process2(Mat src2, Mat src)	//原车道图和预处理后对应的车道图
{
	Mat dst;
	Point2f srcPoint[4], dstPoint[4];
	//src = imread(filename2, IMREAD_GRAYSCALE);
	dst = Mat::zeros(src.size(), src.type());
	srcPoint[0] = Point2f(src.cols / 2 - 63, src.rows / 2 + 100);
	srcPoint[1] = Point2f(src.cols / 6 - 20, src.rows);
	srcPoint[2] = Point2f(src.cols * 5 / 6 + 60, src.rows);
	srcPoint[3] = Point2f(src.cols / 2 + 65, src.rows / 2 + 100);

	dstPoint[0] = Point2f(src.cols / 4, 0);
	dstPoint[1] = Point2f(src.cols / 4, src.rows);
	dstPoint[2] = Point2f(src.cols * 3 / 4, src.rows);
	dstPoint[3] = Point2f(src.cols * 3 / 4, 0);
	Mat warp_mat = getPerspectiveTransform(srcPoint, dstPoint);
	warpPerspective(src, dst, warp_mat, src.size());
	imshow("src", src);


	Mat hist_image = Mat::zeros(700, dst.cols, CV_8UC3);
	vector<int> sum;
	Point p1, p2;
	for (int i = 0;i < dst.cols;i++)
	{
		sum.push_back(0);
		for (int j = 0;j < dst.rows;j++)
		{
			if (dst.at<uchar>(j, i) > 200)
				sum[i]++;
		}
		p1.x = i;
		p1.y = hist_image.rows - 1 - sum[i];
		p2.x = i;
		p2.y = hist_image.rows - 1;
		line(hist_image, p1, p2, Scalar(0, 255, 0), 2, LINE_8);
	}
	imshow("hist", hist_image);

	int max_left = 0, max_right = 0;
	//vector<int> base_left, base_right;
	vector<Point2f> base_left, base_right;
	base_left.push_back(Point2f(0, dst.rows - 1));
	base_right.push_back(Point2f(0, dst.rows - 1));
	for (int i = 0;i < dst.cols;i++)
	{
		if (i < dst.cols / 2)
		{
			if (max_left != max(max_left, sum[i]))
			{
				base_left[0].x = i;
				max_left = sum[i];
			}
		}
		else
		{
			if (max_right != max(max_right, sum[i]))
			{
				base_right[0].x = i;
				max_right = sum[i];
			}
		}
	}
	//_cprintf("%d\t%d\n", base_left, base_right);

	const int win_num = 20;
	Mat dst2;
	cvtColor(dst, dst2, COLOR_GRAY2BGR);
	WinSlide(dst2, &base_left, win_num, Scalar(255, 0, 0));
	WinSlide(dst2, &base_right, win_num, Scalar(0, 0, 255));

	_cprintf("size = %d\n", base_left.size());
	const int poly_n = 2; // 二次多项式拟合
	double coefficients_x[poly_n + 1], coefficients_y[poly_n + 1];// 定义系数数组
	double a[5];
	//vector<double> x(win_num), y(win_num);
	vector<double> x_left, y_left;
	vector<double> x_right, y_right;
	for (int i = 0;i < win_num;i++)
	{
		x_left.push_back(base_left[i].x);
		y_left.push_back(base_left[i].y);
		x_right.push_back(base_right[i].x);
		y_right.push_back(base_right[i].y);
		//circle(dst2, Point(x[i], y[i]), 3, Scalar(255, 255, 0), 2);
	}
	// 多项式拟合
	polyfit(win_num, y_left, x_left, poly_n, coefficients_x);	//拟合式x,y关系与实际坐标系相反
	polyfit(win_num, y_right, x_right, poly_n, coefficients_y);
	_cprintf("y_lefy = %f + %fx + %fx^2\n", coefficients_x[0], coefficients_x[1], coefficients_x[2]);//对应图像坐标中其实是横轴坐标和纵轴坐标的关系
	_cprintf("y_right = %f + %fx + %fx^2\n", coefficients_y[0], coefficients_y[1], coefficients_y[2]);

	//绘制出拟合的二次曲线车道线
	vector<Point> poly_left, poly_right;
	double y_left_buff, y_right_buff;
	for (int i = 0;i < dst2.rows;i++)
	{
		y_left_buff = coefficients_x[0] + coefficients_x[1] * i + coefficients_x[2] * i * i;
		y_right_buff = coefficients_y[0] + coefficients_y[1] * i + coefficients_y[2] * i * i;
		//_cprintf("%d\t", y_buff);
		poly_left.push_back(Point(saturate_cast<int>(y_left_buff), i));
		poly_right.push_back(Point(saturate_cast<int>(y_right_buff), i));
		//_cprintf("x=%d,y=%d\t", poly_left[i].x, poly_left[i].x);
	}
	vector<vector<Point>> contours;
	Mat dst3 = dst2.clone();
	contours.push_back(poly_left);
	contours.push_back(poly_right);
	cv::polylines(dst3, contours, false, Scalar(0, 255, 255), 6, LINE_AA);
	imshow("滑窗拟合", dst3);

	vector<Point> fill;
	Mat dst4;
	cvtColor(dst, dst4, COLOR_GRAY2BGR);
	fill.insert(fill.end(), poly_left.begin(), poly_left.end());
	fill.insert(fill.end(), poly_right.rbegin(), poly_right.rend());
	fillPoly(dst4, fill, Scalar(0, 255, 0));
	imshow("车道区域", dst4);

	Mat dst4_buff;
	Mat output;
	/*src2 = imread(filename1);*/
	Mat warp_mat2 = getPerspectiveTransform(dstPoint, srcPoint);
	warpPerspective(dst4, dst4_buff, warp_mat2, dst4.size());
	addWeighted(src2, 1, dst4_buff, 0.5, 0, output);

	imshow("dst", dst);
	imshow("dst2", dst2);
	imshow("逆投影到原图", output);
}

void CarLineVideo(String filename)
{
	VideoCapture capture(filename);
	if (!capture.isOpened())
	{
		_cprintf("Can't open the video!\n");
		return;
	}
	while (1)
	{
		Mat fram;
		if(!capture.read(fram))
			break;
		//imshow("车道线视频1", fram);
		Mat fram_detection;
		fram_detection = CarLineDetection(fram);
		imshow("车道线视频", fram_detection);
		if (waitKey(1) == 'q')
		{
			destroyAllWindows();
			break;
		}
		//waitKey(30);
	}
	capture.release();
}

Mat CarLineDetection(Mat src_BGR)
{
	//预处理部分
	Mat dst_preprocess, src_HLS, dst_L, dst_S;
	Mat channels[3];
	ThresholdSeparation fixed;
	cvtColor(src_BGR, src_HLS, COLOR_BGR2HLS);
	split(src_HLS, channels);
	//dst_L = SobelEdgeDetection(channels[1]);
	Sobel(channels[1], dst_L, -1, 1, 0);
	//dst_S = fixed.KitlteThreshold(channels[2]);
	inRange(channels[2], Scalar(170, 0, 0), Scalar(255, 0, 0), dst_S);
	dst_preprocess = dst_L + dst_S;

	/*imshow("src", src_BGR);
	imshow("Sobel_L", dst_L);
	imshow("Threshold_S", dst_S);
	imshow("dst", dst_preprocess);*/

	//检测部分
	Mat dst;
	Point2f srcPoint[4], dstPoint[4];
	//src = imread(filename2, IMREAD_GRAYSCALE);
	dst = Mat::zeros(dst_preprocess.size(), dst_preprocess.type());
	srcPoint[0] = Point2f(dst_preprocess.cols / 2 - 63, dst_preprocess.rows / 2 + 100);
	srcPoint[1] = Point2f(dst_preprocess.cols / 6 - 20, dst_preprocess.rows);
	srcPoint[2] = Point2f(dst_preprocess.cols * 5 / 6 + 60, dst_preprocess.rows);
	srcPoint[3] = Point2f(dst_preprocess.cols / 2 + 65, dst_preprocess.rows / 2 + 100);

	dstPoint[0] = Point2f(dst_preprocess.cols / 4, 0);
	dstPoint[1] = Point2f(dst_preprocess.cols / 4, dst_preprocess.rows);
	dstPoint[2] = Point2f(dst_preprocess.cols * 3 / 4, dst_preprocess.rows);
	dstPoint[3] = Point2f(dst_preprocess.cols * 3 / 4, 0);
	Mat warp_mat = getPerspectiveTransform(srcPoint, dstPoint);
	warpPerspective(dst_preprocess, dst, warp_mat, dst_preprocess.size());


	Mat hist_image = Mat::zeros(700, dst.cols, CV_8UC3);
	vector<int> sum;
	Point p1, p2;
	for (int i = 0; i < dst.cols; i++)
	{
		sum.push_back(0);
		for (int j = 0; j < dst.rows; j++)
		{
			if (dst.at<uchar>(j, i) > 200)
				sum[i]++;
		}
		p1.x = i;
		p1.y = hist_image.rows - 1 - sum[i];
		p2.x = i;
		p2.y = hist_image.rows - 1;
		line(hist_image, p1, p2, Scalar(0, 255, 0), 2, LINE_8);
	}
	//imshow("hist", hist_image);

	int max_left = 0, max_right = 0;
	//vector<int> base_left, base_right;
	vector<Point2f> base_left, base_right;
	base_left.push_back(Point2f(0, dst.rows - 1));
	base_right.push_back(Point2f(0, dst.rows - 1));
	for (int i = 0; i < dst.cols; i++)
	{
		if (i < dst.cols / 2)
		{
			if (max_left != max(max_left, sum[i]))
			{
				base_left[0].x = i;
				max_left = sum[i];
			}
		}
		else
		{
			if (max_right != max(max_right, sum[i]))
			{
				base_right[0].x = i;
				max_right = sum[i];
			}
		}
	}
	//_cprintf("%d\t%d\n", base_left, base_right);

	const int win_num = 20;
	Mat dst2;
	cvtColor(dst, dst2, COLOR_GRAY2BGR);
	WinSlide(dst2, &base_left, win_num, Scalar(255, 0, 0));
	WinSlide(dst2, &base_right, win_num, Scalar(0, 0, 255));

	//_cprintf("size = %d\n", base_left.size());
	const int poly_n = 2; // 二次多项式拟合
	double coefficients_x[poly_n + 1], coefficients_y[poly_n + 1];// 定义系数数组
	double a[5];
	//vector<double> x(win_num), y(win_num);
	vector<double> x_left, y_left;
	vector<double> x_right, y_right;
	for (int i = 0; i < win_num; i++)
	{
		x_left.push_back(base_left[i].x);
		y_left.push_back(base_left[i].y);
		x_right.push_back(base_right[i].x);
		y_right.push_back(base_right[i].y);
		//circle(dst2, Point(x[i], y[i]), 3, Scalar(255, 255, 0), 2);
	}
	// 多项式拟合
	polyfit(win_num, y_left, x_left, poly_n, coefficients_x);	//拟合式x,y关系与实际坐标系相反
	polyfit(win_num, y_right, x_right, poly_n, coefficients_y);
	//_cprintf("y_lefy = %f + %fx + %fx^2\n", coefficients_x[0], coefficients_x[1], coefficients_x[2]);//对应图像坐标中其实是横轴坐标和纵轴坐标的关系
	//_cprintf("y_right = %f + %fx + %fx^2\n", coefficients_y[0], coefficients_y[1], coefficients_y[2]);

	//绘制出拟合的二次曲线车道线
	vector<Point> poly_left, poly_right;
	double y_left_buff, y_right_buff;
	for (int i = 0; i < dst2.rows; i++)
	{
		y_left_buff = coefficients_x[0] + coefficients_x[1] * i + coefficients_x[2] * i * i;
		y_right_buff = coefficients_y[0] + coefficients_y[1] * i + coefficients_y[2] * i * i;
		//_cprintf("%d\t", y_buff);
		poly_left.push_back(Point(saturate_cast<int>(y_left_buff), i));
		poly_right.push_back(Point(saturate_cast<int>(y_right_buff), i));
		//_cprintf("x=%d,y=%d\t", poly_left[i].x, poly_left[i].x);
	}
	vector<vector<Point>> contours;
	Mat dst3 = dst2.clone();
	contours.push_back(poly_left);
	contours.push_back(poly_right);
	cv::polylines(dst3, contours, false, Scalar(0, 255, 255), 6, LINE_AA);
	//imshow("滑窗拟合", dst3);

	vector<Point> fill;
	Mat dst4;
	cvtColor(dst, dst4, COLOR_GRAY2BGR);
	fill.insert(fill.end(), poly_left.begin(), poly_left.end());
	fill.insert(fill.end(), poly_right.rbegin(), poly_right.rend());
	fillPoly(dst4, fill, Scalar(0, 255, 0));
	//imshow("车道区域", dst4);

	Mat dst4_buff;
	Mat output;
	/*src2 = imread(filename1);*/
	Mat warp_mat2 = getPerspectiveTransform(dstPoint, srcPoint);
	warpPerspective(dst4, dst4_buff, warp_mat2, dst4.size());
	addWeighted(src_BGR, 1, dst4_buff, 0.5, 0, output);

	/*imshow("dst", dst);
	imshow("dst2", dst2);*/
	//imshow("逆投影到原图", output);
	return output;
}

/*针对车道线图像，进行边缘检测，通过hough变换检测出车道线，并显示*/
void HoughLineDetection(String filename)
{
	Mat src = imread(filename);
	Mat src_gray;
	Mat edges;
	cvtColor(src, src_gray, COLOR_BGR2GRAY);
	Canny(src_gray, edges, 50, 200);
	imshow("原始图", src);
	imshow("Canny边缘检测图", edges);

	// Hough 变换检测直线
	vector<Vec2f> lines;
	HoughLines(edges, lines, 1, CV_PI / 180, 200);
	// 绘制检测到的直线
	Mat result;
	result = src.clone();
	//cvtColor(edges, result, COLOR_GRAY2BGR);
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0];
		float theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		cv::line(result, pt1, pt2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
	}
	imshow("效果图", result);
}

void WinSlide(Mat output_image, vector<Point2f>* base_point, int win_num, Scalar markcolor)
{
	int base_location = (*base_point)[0].x;
	Point left_up;
	Point right_bottom;
	for (int k = 0;k < win_num - 1;k++)
	{
		left_up.x = (*base_point)[k].x - 60;
		left_up.y = (output_image.rows - 1) - (output_image.rows / win_num) * (k + 1);
		right_bottom.x = (*base_point)[k].x + 60;
		right_bottom.y = (output_image.rows - 1) - (output_image.rows / win_num) * (k);
		rectangle(output_image, left_up, right_bottom, Scalar(0, 255, 0));

		int win_sum = 0;	// 统计测量窗口内有多少像素
		int win_max = 0;	//窗口像素阈值，如果像素量达到某个阈值，它将下一个窗口移动到检测到的像素的平均横向位置。
							//如果检测到的像素不足，则下一个窗口将在同一横向位置开始。
		int x_sum = 0;
		Mat img_buff;
		cvtColor(output_image, img_buff, COLOR_BGR2GRAY);
		//imshow("img_buff", img_buff);
		for (int i = left_up.x;i < right_bottom.x;i++)
			for (int j = left_up.y;j < right_bottom.y;j++)
			{
				if (img_buff.at<uchar>(j, i) > 150)
				{
					win_sum++;
					x_sum += i;
					if (base_location < output_image.cols)
						output_image.at<Vec3b>(j, i) = Vec3b(markcolor[0], markcolor[1], markcolor[2]);
				}
			}
		//_cprintf("win_sum = %d\tx_sum = %d\n", win_sum, x_sum);
		if (win_sum > win_max)
			(*base_point).push_back(Point2f(x_sum / win_sum, left_up.y - (output_image.rows / win_num) / 2));
		else
			(*base_point).push_back((*base_point)[k]);
	}
}

//曲线拟合函数：
void polyfit(int n, vector <double> x, vector <double> y, int poly_n, double a[])//点数，X、Y坐标，次数，系数
{
	int i, j;
	double* tempx, * tempy, * sumxx, * sumxy, * ata;

	tempx = new double[n];
	sumxx = new double[poly_n * 2 + 1];
	tempy = new double[n];
	sumxy = new double[poly_n + 1];
	ata = new double[(poly_n + 1) * (poly_n + 1)];

	for (i = 0; i < n; i++)
	{
		tempx[i] = 1;
		tempy[i] = y[i];
	}
	for (i = 0; i < 2 * poly_n + 1; i++)
		for (sumxx[i] = 0, j = 0; j < n; j++)
		{
			sumxx[i] += tempx[j];
			tempx[j] *= x[j];
		}
	for (i = 0; i < poly_n + 1; i++)
		for (sumxy[i] = 0, j = 0;j < n;j++)
		{
			sumxy[i] += tempy[j];
			tempy[j] *= x[j];
		}
	for (i = 0;i < poly_n + 1;i++)
		for (j = 0;j < poly_n + 1;j++)
			ata[i * (poly_n + 1) + j] = sumxx[i + j];
	gauss_solve(poly_n + 1, ata, a, sumxy);

	delete[] tempx;
	tempx = NULL;
	delete[] sumxx;
	sumxx = NULL;
	delete[] tempy;
	tempy = NULL;
	delete[] sumxy;
	sumxy = NULL;
	delete[] ata;
	ata = NULL;
}

void gauss_solve(int n, double A[], double x[], double b[])
{
	int i, j, k, r;
	double max;
	for (k = 0;k < n - 1;k++)
	{
		max = fabs(A[k * n + k]); /*find maxmum*/
		r = k;
		for (i = k + 1;i < n - 1;i++)
			if (max < fabs(A[i * n + i]))
			{
				max = fabs(A[i * n + i]);
				r = i;
			}
		if (r != k)
			for (i = 0;i < n;i++) /*change array:A[k]&A[r] */
			{
				max = A[k * n + i];
				A[k * n + i] = A[r * n + i];
				A[r * n + i] = max;
			}
		max = b[k]; /*change array:b[k]&b[r] */
		b[k] = b[r];
		b[r] = max;
		for (i = k + 1;i < n;i++)
		{
			for (j = k + 1;j < n;j++)
				A[i * n + j] -= A[i * n + k] * A[k * n + j] / A[k * n + k];
			b[i] -= A[i * n + k] * b[k] / A[k * n + k];
		}
	}

	for (i = n - 1;i >= 0;x[i] /= A[i * n + i], i--)
		for (j = i + 1, x[i] = b[i];j < n;j++)
			x[i] -= A[i * n + j] * x[j];
}
