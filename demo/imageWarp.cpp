#include "pch.h"
#include "image.h"

void ImageWarpAffine(String filename)
{
	Point2f srcPoint[3], dstPoint[3];
	Mat src, dst;
	Mat warp_mat;
	src = imread(filename, IMREAD_COLOR);
	dst = Mat::zeros(src.size(), src.type());
	srcPoint[0] = Point2f( 0,0 );
	srcPoint[1] = Point2f( src.cols - 1,0 );
	srcPoint[2] = Point2f( 0,src.rows - 1 );
	dstPoint[0] = Point2f( 0.0,src.rows * 0.33 );
	dstPoint[1] = Point2f( src.cols * 0.85,src.rows * 0.25 );
	dstPoint[2] = Point2f( src.cols * 0.15,src.rows * 0.7 );
	warp_mat = getAffineTransform(srcPoint, dstPoint);
	warpAffine(src, dst, warp_mat, src.size());
	imshow("src", src);
	imshow("wrapAffine_image", dst);
}

void ImageWrapPerspective(Mat src, Point2f* srcPoint, Point2f* dstPoint, Size dst_size)
{
	Mat dst;
	dst = Mat::zeros(src.size(), src.type());
	Mat wrap_mat;
	wrap_mat = getPerspectiveTransform(srcPoint, dstPoint);
	warpPerspective(src, dst, wrap_mat, dst_size);
	imshow("src", src);
	imshow("warpPerspective_image", dst);
}

