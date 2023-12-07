#include "pch.h"
#include "image.h"

void Brisk_BlemishDetection(String filename1, String filename2)
{
	//BRISK特征点提取
	Mat src1, src2;
	Mat des1, des2;
	Mat res1, res2;
	src1 = imread(filename1, 1);
	src2 = imread(filename2, 1);
	Ptr<BRISK> detector = BRISK::create();
	vector<KeyPoint> kp1, kp2;
	detector->detect(src1, kp1);
	detector->detect(src2, kp2);
	detector->compute(src1, kp1, des1);
	detector->compute(src2, kp2, des2);
	drawKeypoints(src1, kp1, res1, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	drawKeypoints(src2, kp2, res2, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	//暴力匹配_汉明距离法
	BFMatcher matcher(NORM_HAMMING);
	vector<DMatch> matches;
	matcher.match(des1, des2, matches);
	Mat img_match;
	drawMatches(src1, kp1, src2, kp2, matches, img_match);
	imshow("matches", img_match);

	//消除匹配错点
	vector<int> queryIdxs(matches.size()), trainIdxs(matches.size());
	for (size_t i = 0; i < matches.size(); i++) {
		queryIdxs[i] = matches[i].queryIdx;
		trainIdxs[i] = matches[i].trainIdx;
	}
	Mat H12;
	vector<Point2f> points1;
	KeyPoint::convert(kp1, points1, queryIdxs);
	vector<Point2f> points2;
	KeyPoint::convert(kp2, points2, trainIdxs);
	int ransacReprojThreshold = 5;

	H12 = findHomography(Mat(points1), Mat(points2), RANSAC, ransacReprojThreshold);
	vector<char> matchesMask(matches.size(), 0);
	Mat points1t;
	perspectiveTransform(Mat(points1), points1t, H12);
	for (size_t i1 = 0; i1 < points1.size(); i1++) if (norm(points2[i1] - points1t.at<Point2f>((int)i1, 0)) <= ransacReprojThreshold) matchesMask[i1] = 1;
	Mat match_img2;
	drawMatches(src1, kp1, src2, kp2, matches, match_img2, Scalar(0, 0, 255), Scalar::all(-1), matchesMask);

	//画出目标位置
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = Point(0, 0);
	obj_corners[1] = Point(src1.cols, 0);
	obj_corners[2] = Point(src1.cols, src1.rows);
	obj_corners[3] = Point(0, src1.rows);
	vector<Point2f> scene_corners(4);
	perspectiveTransform(obj_corners, scene_corners, H12);
	for (int i = 0; i < 4; i++) line(match_img2, Point2f((scene_corners[i % 4].x + static_cast<float>(src1.cols)), (scene_corners[i % 4].y)),
		Point2f((scene_corners[(i + 1) % 4].x + static_cast<float>(src1.cols)), (scene_corners[(i + 1) % 4].y)), Scalar(0, 0, 255), 2, 8, 0);
	imshow("matches_new", match_img2);

	//寻找瑕疵
	Point2f src_point[4];
	Point2f dst_point[4];
	src_point[0].x = scene_corners[0].x;
	src_point[0].y = scene_corners[0].y;
	src_point[1].x = scene_corners[1].x;
	src_point[1].y = scene_corners[1].y;
	src_point[2].x = scene_corners[2].x;
	src_point[2].y = scene_corners[2].y;
	src_point[3].x = scene_corners[3].x;
	src_point[3].y = scene_corners[3].y;

	dst_point[0].x = 0;
	dst_point[0].y = 0;
	dst_point[1].x = src1.cols;
	dst_point[1].y = 0;
	dst_point[2].x = src1.cols;
	dst_point[2].y = src1.rows;
	dst_point[3].x = 0;
	dst_point[3].y = src1.rows;

	Mat newM(3, 3, CV_32FC1);
	newM = getPerspectiveTransform(src_point, dst_point);
	Mat dst = src2.clone();
	warpPerspective(src2, dst, newM, src1.size());
	imshow("dst", dst);
	Mat resultimg = dst.clone();
	absdiff(src1, dst, resultimg);
	imshow("resultimg", resultimg);

	waitKey();
}

void ORB_BlemishDetection(String filename1, String filename2)
{
	Mat obj = imread(filename1);   //载入目标图像
	Mat scene = imread(filename2); //载入场景图像
	if (obj.empty() || scene.empty())
	{
		cout << "Can't open the picture!\n";
		return;
	}
	vector<KeyPoint> obj_keypoints, scene_keypoints;
	Mat obj_descriptors, scene_descriptors;
	Ptr<ORB> detector = ORB::create();

	detector->detect(obj, obj_keypoints);
	detector->detect(scene, scene_keypoints);
	detector->compute(obj, obj_keypoints, obj_descriptors);
	detector->compute(scene, scene_keypoints, scene_descriptors);

	BFMatcher matcher(NORM_HAMMING, true); //汉明距离做为相似度度量
	vector<DMatch> matches;
	matcher.match(obj_descriptors, scene_descriptors, matches);
	Mat match_img;
	drawMatches(obj, obj_keypoints, scene, scene_keypoints, matches, match_img);
	imshow("滤除误匹配前", match_img);

	//保存匹配对序号
	vector<int> queryIdxs(matches.size()), trainIdxs(matches.size());
	for (size_t i = 0; i < matches.size(); i++)
	{
		queryIdxs[i] = matches[i].queryIdx;
		trainIdxs[i] = matches[i].trainIdx;
	}

	Mat H12;   //变换矩阵

	vector<Point2f> points1;
	KeyPoint::convert(obj_keypoints, points1, queryIdxs);
	vector<Point2f> points2;
	KeyPoint::convert(scene_keypoints, points2, trainIdxs);
	int ransacReprojThreshold = 5;  //拒绝阈值


	H12 = findHomography(Mat(points1), Mat(points2), RANSAC, ransacReprojThreshold);
	vector<char> matchesMask(matches.size(), 0);
	Mat points1t;
	perspectiveTransform(Mat(points1), points1t, H12);
	for (size_t i1 = 0; i1 < points1.size(); i1++)  //保存‘内点’
	{
		if (norm(points2[i1] - points1t.at<Point2f>((int)i1, 0)) <= ransacReprojThreshold) //给内点做标记
		{
			matchesMask[i1] = 1;
		}
	}
	Mat match_img2;   //滤除‘外点’后
	drawMatches(obj, obj_keypoints, scene, scene_keypoints, matches, match_img2, Scalar(0, 0, 255), Scalar::all(-1), matchesMask);

	//画出目标位置
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = Point(0, 0); obj_corners[1] = Point(obj.cols, 0);
	obj_corners[2] = Point(obj.cols, obj.rows); obj_corners[3] = Point(0, obj.rows);
	std::vector<Point2f> scene_corners(4);
	perspectiveTransform(obj_corners, scene_corners, H12);
	//line( match_img2, scene_corners[0] + Point2f(static_cast<float>(obj.cols), 0),scene_corners[1] + Point2f(static_cast<float>(obj.cols), 0),Scalar(0,0,255),2);
	//line( match_img2, scene_corners[1] + Point2f(static_cast<float>(obj.cols), 0),scene_corners[2] + Point2f(static_cast<float>(obj.cols), 0),Scalar(0,0,255),2);
	//line( match_img2, scene_corners[2] + Point2f(static_cast<float>(obj.cols), 0),scene_corners[3] + Point2f(static_cast<float>(obj.cols), 0),Scalar(0,0,255),2);
	//line( match_img2, scene_corners[3] + Point2f(static_cast<float>(obj.cols), 0),scene_corners[0] + Point2f(static_cast<float>(obj.cols), 0),Scalar(0,0,255),2);
	line(match_img2, Point2f((scene_corners[0].x + static_cast<float>(obj.cols)), (scene_corners[0].y)), Point2f((scene_corners[1].x + static_cast<float>(obj.cols)), (scene_corners[1].y)), Scalar(0, 0, 255), 2);
	line(match_img2, Point2f((scene_corners[1].x + static_cast<float>(obj.cols)), (scene_corners[1].y)), Point2f((scene_corners[2].x + static_cast<float>(obj.cols)), (scene_corners[2].y)), Scalar(0, 0, 255), 2);
	line(match_img2, Point2f((scene_corners[2].x + static_cast<float>(obj.cols)), (scene_corners[2].y)), Point2f((scene_corners[3].x + static_cast<float>(obj.cols)), (scene_corners[3].y)), Scalar(0, 0, 255), 2);
	line(match_img2, Point2f((scene_corners[3].x + static_cast<float>(obj.cols)), (scene_corners[3].y)), Point2f((scene_corners[0].x + static_cast<float>(obj.cols)), (scene_corners[0].y)), Scalar(0, 0, 255), 2);

	float A_th;
	A_th = atan(abs((scene_corners[3].y - scene_corners[0].y) / (scene_corners[3].x - scene_corners[0].x)));
	A_th = 90 - 180 * A_th / 3.14;
	_cprintf("angle=%f\n", A_th);

	imshow("滤除误匹配后", match_img2);

	//line( scene, scene_corners[0],scene_corners[1],Scalar(0,0,255),2);
	//line( scene, scene_corners[1],scene_corners[2],Scalar(0,0,255),2);
	//line( scene, scene_corners[2],scene_corners[3],Scalar(0,0,255),2);
	//line( scene, scene_corners[3],scene_corners[0],Scalar(0,0,255),2);

	imshow("场景图像", scene);

	Mat rotimage;
	Mat rotate = getRotationMatrix2D(Point(scene.cols / 2, scene.rows / 2), A_th, 1);
	warpAffine(scene, rotimage, rotate, scene.size());
	imshow("rotimage", rotimage);


	//方法三 透视变换  
	Point2f src_point[4];
	Point2f dst_point[4];
	src_point[0].x = scene_corners[0].x;
	src_point[0].y = scene_corners[0].y;
	src_point[1].x = scene_corners[1].x;
	src_point[1].y = scene_corners[1].y;
	src_point[2].x = scene_corners[2].x;
	src_point[2].y = scene_corners[2].y;
	src_point[3].x = scene_corners[3].x;
	src_point[3].y = scene_corners[3].y;


	dst_point[0].x = 0;
	dst_point[0].y = 0;
	dst_point[1].x = obj.cols;
	dst_point[1].y = 0;
	dst_point[2].x = obj.cols;
	dst_point[2].y = obj.rows;
	dst_point[3].x = 0;
	dst_point[3].y = obj.rows;

	Mat newM(3, 3, CV_32FC1);
	newM = getPerspectiveTransform(src_point, dst_point);

	Mat dst = scene.clone();

	warpPerspective(scene, dst, newM, obj.size());

	imshow("obj", obj);
	imshow("dst", dst);

	Mat resultimg = dst.clone();

	absdiff(obj, dst, resultimg);//当前帧跟前面帧相减

	imshow("result", resultimg);

	imshow("dst", dst);
	imshow("src", obj);
}