#include "pch.h"
#include "image.h"

void stereo_match(int, void*);

/******** ��Ŀ����궨���� ************/
void SignalCamera()
{
	ifstream fin(".\\data\\camer_cab\\imageDatalist_right.txt"); /* �궨����ͼ���ļ���·�� */
	ofstream fout(".\\data\\camer_cab\\caliberation_result_right.txt");  /* ����궨������ļ� */
											//��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��	
	_cprintf("start to find corners������������\n");
	int image_count = 0;  /* ͼ������ */
	Size image_size;  /* ͼ��ĳߴ� */
	Size board_size = Size(9, 6);    /* �궨����ÿ�С��еĽǵ��� */
	vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */
	string filename;
	int count = -1;//���ڴ洢�ǵ������
	while (getline(fin, filename))
	{
		image_count++;
		// ���ڹ۲�������
		_cprintf("image_count = %d\n", image_count);

		Mat imageInput = imread(filename);
		if (image_count == 1)  //�����һ��ͼƬʱ��ȡͼ�������Ϣ
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			_cprintf("image_size.width = %d\n", image_size.width);
			_cprintf("image_size.height = %d\n", image_size.height);
		}

		/* ��ȡ�ǵ� */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			_cprintf("can not find chessboard corners!\n"); //�Ҳ����ǵ�
			waitKey(0);
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imageInput, view_gray, COLOR_RGBA2GRAY);
			/* �����ؾ�ȷ�� */
			find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ��
																			//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
														   /* ��ͼ������ʾ�ǵ�λ�� */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); //������ͼƬ�б�ǽǵ�
			imshow("Camera Calibration", view_gray);//��ʾͼƬ
			waitKey(50);//��ͣ0.5S		
		}
	}
	int total = image_points_seq.size();
	_cprintf("total = %d\n", total);
	int CornerNum = board_size.width * board_size.height;  //ÿ��ͼƬ���ܵĽǵ���
	for (int ii = 0; ii < total; ii++)
	{
		if (0 == ii % CornerNum)// 24 ��ÿ��ͼƬ�Ľǵ���������ж������Ϊ����� ͼƬ�ţ����ڿ���̨�ۿ� 
		{
			int i = -1;
			i = ii / CornerNum;
			int j = i + 1;
			_cprintf("--> the %d image's data --> :", j);
		}
		if (0 == ii % 3)	// ���ж���䣬��ʽ����������ڿ���̨�鿴
		{
			_cprintf("\n");
		}
		else
		{
			cout.width(10);
		}
		//������еĽǵ�
		_cprintf(" -->%f", image_points_seq[ii][0].x);
		_cprintf(" -->%f", image_points_seq[ii][0].y);
	}
	_cprintf("find corner successfully��\n");

	//������������궨
	_cprintf("start calibration������������");
	/*������ά��Ϣ*/
	Size square_size = Size(10, 10);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
	vector<vector<Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
										   /*�������*/
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ������ڲ������� */
	vector<int> point_counts;  // ÿ��ͼ���нǵ������
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */
	vector<Mat> tvecsMat;  /* ÿ��ͼ�����ת���� */
	vector<Mat> rvecsMat; /* ÿ��ͼ���ƽ������ */
						  /* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (t = 0; t < image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				Point3f realPoint;
				/* ����궨�������������ϵ��z=0��ƽ���� */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
	for (i = 0; i < image_count; i++)
	{
		point_counts.push_back(board_size.width * board_size.height);
	}
	/* ��ʼ�궨 */
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	_cprintf("calibrate successfully��\n");
	//�Ա궨�����������
	_cprintf("start evaluate the calibration results������������\n");
	double total_err = 0.0; /* ����ͼ���ƽ�������ܺ� */
	double err = 0.0; /* ÿ��ͼ���ƽ����� */
	vector<Point2f> image_points2; /* �������¼���õ���ͶӰ�� */
	_cprintf("\t each image's calibration error��\n");
	_cprintf("each image's calibration error��\n");
	for (i = 0; i < image_count; i++)
	{
		vector<Point3f> tempPointSet = object_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		_cprintf("the %d th image's average err: %f pixels\n", i + 1, err);
		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
	_cprintf(" the total average err��%f \n", total_err / image_count);
	fout << "����ƽ����" << total_err / image_count << "����" << endl << endl;
	_cprintf("evaluate successfully��\n");
	//���涨����  	
	_cprintf("start save calibration results������������\n");
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ����ÿ��ͼ�����ת���� */
	fout << "����ڲ�������" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "����ϵ����\n";
	fout << distCoeffs << endl << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
		fout << tvecsMat[i] << endl;
		/* ����ת����ת��Ϊ���Ӧ����ת���� */
		Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
		fout << rotation_matrix << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
		fout << rvecsMat[i] << endl << endl;
	}
	_cprintf("save successfully\n");
	fout << endl;
	/************************************************************************
	��ʾ������
	*************************************************************************/
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	_cprintf("save rectified image\n");
	string imageFileName;
	std::stringstream StrStm;
	for (int i = 0; i != image_count; i++)
	{
		_cprintf("Frame # %d ...\n", i + 1);
		initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
		StrStm.clear();
		imageFileName.clear();
		string filePath = ".//data//camer_cab//chess";
		StrStm << i + 1;
		StrStm >> imageFileName;
		filePath += imageFileName;
		filePath += ".jpg";
		Mat imageSource = imread(".//data//camer_cab//left01.jpg");
		Mat newimage = imageSource.clone();
		//��һ�ֲ���Ҫת������ķ�ʽ
		//undistort(imageSource,newimage,cameraMatrix,distCoeffs);
		remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);
		StrStm.clear();
		filePath.clear();
		filePath = ".//data//camer_cab//chess";
		StrStm << i + 1;
		StrStm >> imageFileName;
		filePath += imageFileName;
		filePath += "_d.jpg";
		imwrite(filePath, newimage);
	}
	_cprintf(" save successfully\n");
	waitKey(0);
}


/******** ˫Ŀ����궨���� ************/
/*����궨����ģ���ʵ����������*/
void calRealPoint(vector<vector<Point3f>>& obj, int boardWidth, int boardHeight, int imgNumber, int squareSize)
{
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardHeight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardWidth; colIndex++)
		{
			imgpoint.push_back(Point3f(rowIndex * squareSize, colIndex * squareSize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}
Mat R, T, E, F;
Mat Rl, Rr, Pl, Pr, Q;
//ӳ���
Mat mapLx, mapLy, mapRx, mapRy;

Mat cameraMatrixL = (Mat_<double>(3, 3) << 530.1397548683084, 0, 338.2680507680664,
	0, 530.2291152852337, 232.4902023212199,
	0, 0, 1);
//��õĻ������
Mat distCoeffL = (Mat_<double>(5, 1) << -0.266294943795012, -0.0450330886310585, 0.0003024821418382528, -0.001243865371699451, 0.2973605735168139);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 530.1397548683084, 0, 338.2680507680664,
	0, 530.2291152852337, 232.4902023212199,
	0, 0, 1);
Mat distCoeffR = (Mat_<double>(5, 1) << -0.266294943795012, -0.0450330886310585, 0.0003024821418382528, -0.001243865371699451, 0.2973605735168139);
void outputCameraParam(void)
{
	/*��������*/
	/*�������*/
	FileStorage fs("intrisics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "cameraMatrixL" << cameraMatrixL << "cameraDistcoeffL" << distCoeffL << "cameraMatrixR" << cameraMatrixR << "cameraDistcoeffR" << distCoeffR;
		fs.release();
		cout << "cameraMatrixL=:" << cameraMatrixL << endl << "cameraDistcoeffL=:" << distCoeffL << endl << "cameraMatrixR=:" << cameraMatrixR << endl << "cameraDistcoeffR=:" << distCoeffR << endl;
	}
	else
	{
		cout << "Error: can not save the intrinsics!!!!" << endl;
	}

	fs.open("extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr << "Q" << Q;
		cout << "R=" << R << endl << "T=" << T << endl << "Rl=" << Rl << endl << "Rr" << Rr << endl << "Pl" << Pl << endl << "Pr" << Pr << endl << "Q" << Q << endl;
		fs.release();
	}
	else
	{
		cout << "Error: can not save the extrinsic parameters\n";
	}
}

void DoubleCamera()
{
	//����ͷ�ķֱ���
	const int imageWidth = 640;
	const int imageHeight = 480;
	//����Ľǵ���Ŀ
	const int boardWidth = 9;
	//����Ľǵ���Ŀ
	const int boardHeight = 6;
	//�ܵĽǵ���Ŀ
	const int boardCorner = boardWidth * boardHeight;
	//����궨ʱ��Ҫ���õ�ͼ��֡��
	const int frameNumber = 14;
	//�궨��ڰ׸��ӵĴ�С ��λ��mm
	const int squareSize = 10;
	//�궨������ڽǵ�
	const Size boardSize = Size(boardWidth, boardHeight);
	Size imageSize = Size(imageWidth, imageHeight);


	//R��תʸ�� Tƽ��ʸ�� E�������� F��������
	vector<Mat> rvecs; //R
	vector<Mat> tvecs; //T
					   //��������������Ƭ�ǵ�����꼯��
	vector<vector<Point2f>> imagePointL;
	//�ұ������������Ƭ�ǵ�����꼯��
	vector<vector<Point2f>> imagePointR;
	//��ͼ��Ľǵ��ʵ�ʵ��������꼯��
	vector<vector<Point3f>> objRealPoint;
	//��������ĳһ��Ƭ�ǵ����꼯��
	vector<Point2f> cornerL;
	//�ұ������ĳһ��Ƭ�ǵ����꼯��
	vector<Point2f> cornerR;

	Mat rgbImageL, grayImageL;
	Mat rgbImageR, grayImageR;
	Mat intrinsic;
	Mat distortion_coeff;
	//У����ת����R��ͶӰ����P����ͶӰ����Q
	//ӳ���
	Mat mapLx, mapLy, mapRx, mapRy;
	Rect validROIL, validROIR;
	//ͼ��У��֮�󣬻��ͼ����вü������У�validROI�ü�֮�������

	Mat img;
	int goodFrameCount = 1;
	while (goodFrameCount <= frameNumber)
	{
		char filename[100];
		/*��ȡ��ߵ�ͼ��*/
		sprintf_s(filename, ".//data//camer_cab//left%02d.jpg", goodFrameCount);
		rgbImageL = imread(filename, 1);
		imshow("chessboardL", rgbImageL);
		cvtColor(rgbImageL, grayImageL, COLOR_BGR2GRAY);
		/*��ȡ�ұߵ�ͼ��*/
		sprintf_s(filename, ".//data//camer_cab//right%02d.jpg", goodFrameCount);
		rgbImageR = imread(filename, 1);
		cvtColor(rgbImageR, grayImageR, COLOR_BGR2GRAY);

		bool isFindL, isFindR;
		isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);
		isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);
		if (isFindL == true && isFindR == true)
		{
			cornerSubPix(grayImageL, cornerL, Size(5, 5), Size(-1, 1), TermCriteria(TermCriteria::Type::EPS | TermCriteria::Type::MAX_ITER, 20, 0.1));
			drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);
			imshow("chessboardL", rgbImageL);
			imagePointL.push_back(cornerL);

			cornerSubPix(grayImageR, cornerR, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::Type::EPS | TermCriteria::Type::MAX_ITER, 20, 0.1));
			drawChessboardCorners(rgbImageR, boardSize, cornerR, isFindR);
			imshow("chessboardR", rgbImageR);
			imagePointR.push_back(cornerR);

			_cprintf("the image %d is good\n", goodFrameCount);
			goodFrameCount++;
		}
		else
		{
			_cprintf("the image is bad please try again\n");
		}

		if (waitKey(10) == 'q')
		{
			break;
		}
	}

	//����ʵ�ʵ�У�������ά���꣬����ʵ�ʱ궨���ӵĴ�С������

	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	_cprintf("cal real successful\n");

	//�궨����ͷ
	double rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
		cameraMatrixL, distCoeffL,
		cameraMatrixR, distCoeffR,
		Size(imageWidth, imageHeight), R, T, E, F, CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	_cprintf("Stereo Calibration done with RMS error = %f\n", rms);

	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl,
		Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);

	//�����У��ӳ��
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

	Mat rectifyImageL, rectifyImageR;
	cvtColor(grayImageL, rectifyImageL, COLOR_GRAY2BGR);
	cvtColor(grayImageR, rectifyImageR, COLOR_GRAY2BGR);

	imshow("RecitifyL Before", rectifyImageL);
	imshow("RecitifyR Before", rectifyImageR);

	//����remap֮�����������ͼ���Ѿ����沢���ж�׼��
	Mat rectifyImageL2, rectifyImageR2;
	remap(rectifyImageL, rectifyImageL2, mapLx, mapLy, INTER_LINEAR);
	remap(rectifyImageR, rectifyImageR2, mapRx, mapRy, INTER_LINEAR);


	imshow("rectifyImageL", rectifyImageL2);
	imshow("rectifyImageR", rectifyImageR2);

	outputCameraParam();
	//��ʾУ�����
	Mat canvas;
	double sf;
	int w, h;
	sf = 600. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);

	//��ͼ�񻭵�������
	Mat canvasPart = canvas(Rect(0, 0, w, h));
	resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	Rect vroiL(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),
		cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);

	_cprintf("Painted ImageL\n");

	//��ͼ�񻭵�������
	canvasPart = canvas(Rect(w, 0, w, h));
	resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	_cprintf("Painted ImageR\n");

	//���϶�Ӧ������
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

	imshow("rectified", canvas);
	_cprintf("wait key\n");
	waitKey(0);
}


/******** ����ƥ����� ************/
int getDisparityImage(cv::Mat& disparity, cv::Mat& disparityImage, bool isColor)
{
	cv::Mat disp8u;
	disp8u = disparity;
	// ת��Ϊα��ɫͼ�� �� �Ҷ�ͼ��
	if (isColor)
	{
		if (disparityImage.empty() || disparityImage.type() != CV_8UC3 || disparityImage.size() != disparity.size())
		{
			disparityImage = cv::Mat::zeros(disparity.rows, disparity.cols, CV_8UC3);
		}
		for (int y = 0; y < disparity.rows; y++)
		{
			for (int x = 0; x < disparity.cols; x++)
			{
				uchar val = disp8u.at<uchar>(y, x);
				uchar r, g, b;

				if (val == 0)
					r = g = b = 0;
				else
				{
					r = 255 - val;
					g = val < 128 ? val * 2 : (uchar)((255 - val) * 2);
					b = val;
				}
				disparityImage.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
			}
		}
	}
	else
	{
		disp8u.copyTo(disparityImage);
	}
	return 1;
}
const int imageWidth = 640;                             //����ͷ�ķֱ���  
const int imageHeight = 480;
Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;//ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������  
Rect validROIR;

Mat xyz;              //��ά����
int blockSize = 0, uniquenessRatio = 0, numDisparities = 0;
Ptr<StereoBM> bm = StereoBM::create(16, 9);

Mat T_new = (Mat_<double>(3, 1) << -3.3269653179960471e+01, 3.7375231026230421e-01, -1.2058042444883227e-02);//Tƽ������
//Mat rec = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//rec��ת����
Mat R_new = (Mat_<double>(3, 3) << 9.9998505024526163e-01, 3.5253250461816949e-03,
	4.1798767087380161e-03, -3.4957471578341281e-03,
	9.9996894942320580e-01, -7.0625732745616225e-03,
	-4.2046447876106169e-03, 7.0478558986986593e-03,
	9.9996632377767658e-01);//R ��ת����

/*****����ƥ��*****/
void stereo_match(int, void*)
{
	bm->setBlockSize(2 * blockSize + 5);     //SAD���ڴ�С��5~21֮��Ϊ��
	bm->setROI1(validROIL);
	bm->setROI2(validROIR);
	bm->setPreFilterCap(31);
	bm->setMinDisparity(0);  //��С�ӲĬ��ֵΪ0, �����Ǹ�ֵ��int��
	bm->setNumDisparities(numDisparities * 16 + 16);//�Ӳ�ڣ�������Ӳ�ֵ����С�Ӳ�ֵ֮��,���ڴ�С������16����������int��
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio��Ҫ���Է�ֹ��ƥ��
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(-1);
	Mat disp, disp8, disparityImage;
	bm->compute(rectifyImageL, rectifyImageR, disp);//����ͼ�����Ϊ�Ҷ�ͼ
	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16) * 16.));//��������Ӳ���CV_16S��ʽ
	reprojectImageTo3D(disp, xyz, Q, true); //��ʵ�������ʱ��ReprojectTo3D������X / W, Y / W, Z / W��Ҫ����16(Ҳ����W����16)�����ܵõ���ȷ����ά������Ϣ��
	xyz = xyz * 16;
	getDisparityImage(disp8, disparityImage, true);
	imshow("disparity", disparityImage);
}

//����ƥ��
void StereoMatch()
{
	// TODO: �ڴ����ӿؼ�֪ͨ�����������
	//����У��
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R_new, T_new, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
		0, imageSize, &validROIL, &validROIR);
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

	rgbImageL = imread(".//data//camer_cab//left01.jpg", IMREAD_COLOR);
	cvtColor(rgbImageL, grayImageL,COLOR_BGR2GRAY);
	rgbImageR = imread(".//data//camer_cab//right01.jpg", IMREAD_COLOR);
	cvtColor(rgbImageR, grayImageR, COLOR_BGR2GRAY);

	imshow("ImageL Before Rectify", grayImageL);
	imshow("ImageR Before Rectify", grayImageR);

	/*
	����remap֮�����������ͼ���Ѿ����沢���ж�׼��
	*/
	remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	/*
	��У�������ʾ����
	*/
	Mat rgbRectifyImageL, rgbRectifyImageR;
	cvtColor(rectifyImageL, rgbRectifyImageL, COLOR_GRAY2BGR);  //α��ɫͼ
	cvtColor(rectifyImageR, rgbRectifyImageR, COLOR_GRAY2BGR);
	//������ʾ
	//rectangle(rgbRectifyImageL, validROIL, Scalar(0, 0, 255), 3, 8);
	//rectangle(rgbRectifyImageR, validROIR, Scalar(0, 0, 255), 3, 8);
	imshow("ImageL After Rectify", rgbRectifyImageL);
	imshow("ImageR After Rectify", rgbRectifyImageR);

	//��ʾ��ͬһ��ͼ��
	Mat canvas;
	double sf;
	int w, h;
	sf = 600. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);   //ע��ͨ��

										//��ͼ�񻭵�������
	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //�õ�������һ����  
	resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //��ͼ�����ŵ���canvasPartһ����С  
	Rect vroiL(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),                //��ñ���ȡ������    
		cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
	//rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //����һ������  
	cout << "Painted ImageL" << endl;

	//��ͼ�񻭵�������
	canvasPart = canvas(Rect(w, 0, w, h));                                      //��û�������һ����  
	resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	//rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
	cout << "Painted ImageR" << endl;

	//���϶�Ӧ������
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
	imshow("rectified", canvas);

	/*
	����ƥ��
	*/
	namedWindow("disparity", WINDOW_AUTOSIZE);
	// ����SAD���� Trackbar
	createTrackbar("BlockSize:\n", "disparity", &blockSize, 8, stereo_match);
	// �����Ӳ�Ψһ�԰ٷֱȴ��� Trackbar
	createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);
	// �����Ӳ�� Trackbar
	createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
	stereo_match(0, 0);

	waitKey(0);
}