#include "pch.h"
#include "image.h"

using namespace cv;
using namespace std;

void ImageShow()
{
	Mat img;
	String image_name = ".\\data\\lena.jpg";
	img = imread(image_name, IMREAD_COLOR);
	if (img.empty())
	{
		return;
	}
	imshow("image", img);
	waitKey();
}