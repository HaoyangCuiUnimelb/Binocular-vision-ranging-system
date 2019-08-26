// stereoCalibration.cpp : �������̨Ӧ�ó������ڵ㡣
//
//�ڽ���˫Ŀ����ͷ�ı궨֮ǰ�����ȷֱ����������ͷ���е�Ŀ�Ӿ��ı궨 
//ȷ����������ͷ���ڲξ���Ȼ���ٽ�������궨

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

const int imageWidth = 640;								//����ͷ�ķֱ���
const int imageHeight = 480;
const int boardWidth = 9;								//����Ľǵ���Ŀ
const int boardHeight = 6;								//����Ľǵ�����
const int boardCorner = boardWidth * boardHeight;		//�ܵĽǵ�����
const int frameNumber = 12;								//����궨ʱ��Ҫ���õ�ͼ��֡��
const int squareSize = 25;								//�궨��ڰ׸��ӵĴ�С ��λmm

Size imageSize = Size(imageWidth, imageHeight);

Mat R, T, E, F;											//R ��תʸ�� Tƽ��ʸ�� E�������� F��������
vector<Mat> rvecs;									    //��ת����
vector<Mat> tvecs;										//ƽ������
vector<vector<Point2f>> imagePointL;				    //��������������Ƭ�ǵ�����꼯��
vector<vector<Point2f>> imagePointR;					//�ұ������������Ƭ�ǵ�����꼯��
vector<vector<Point3f>> objRealPoint;					//����ͼ��Ľǵ��ʵ���������꼯��

Mat Rl, Rr, Pl, Pr, Q;									//У����ת����R��ͶӰ����P ��ͶӰ����Q 	
Mat mapLx, mapLy, mapRx, mapRy;							//ӳ���
Rect validROIL, validROIR;								//ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������

Mat cameraMatrixL = (Mat_<double>(3, 3));
Mat distCoeffL = (Mat_<double>(5, 1));
Mat cameraMatrixR = (Mat_<double>(3, 3));
Mat distCoeffR = (Mat_<double>(5, 1));

// /*
// ���ȱ궨�õ���������ڲξ���
// fx 0 cx
// 0 fy cy
// 0 0  1
// */
// Mat cameraMatrixL = (Mat_<double>(3, 3) << 532.782, 0,       532.904,
// 										  0,       342.505, 233.876,
// 										  0,       0,       1);
// Mat distCoeffL = (Mat_<double>(5, 1) << -0.28095, 0.0255745, 0.00122226, -0.000137736, 0.162946);
// /*
// ���ȱ궨�õ���������ڲξ���
// fx 0 cx
// 0 fy cy
// 0 0  1
// */
// Mat cameraMatrixR = (Mat_<double>(3, 3) << 532.782, 0,       532.904,
// 											0,      342.505, 233.876,
// 											0,		0,		 1);
// Mat distCoeffR = (Mat_<double>(5, 1) << -0.28095, 0.0255745, 0.00122226, -0.000137736, 0.162946);

/*����궨����ģ���ʵ����������*/
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

void inputCameraParam(void)
{
	/*��������*/
	FileStorage fs("intrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["cameraMatrixL"] >> cameraMatrixL;
		fs["cameraDistcoeffL"] >> distCoeffL;
		fs["cameraMatrixR"] >> cameraMatrixR;
		fs["cameraDistcoeffR"] >> distCoeffR;
		fs.release();
		cout << "cameraMatrixL=:" << cameraMatrixL << endl << "cameraDistcoeffL=:" << distCoeffL << endl << "cameraMatrixR=:" << cameraMatrixR << endl << "cameraDistcoeffR=:" << distCoeffR << endl;
	}
	else
	{
		cout << "Error: can not read the intrinsics!!!!!" << endl;
	}
}


void outputCameraParam(void)
{
	/*��������*/
	/*�������*/
	FileStorage fs("calib_paras.xml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "imageSize" << imageSize;
		fs << "cameraMatrixL" << cameraMatrixL;
		fs << "cameraDistcoeffL" << distCoeffL;
		fs << "cameraMatrixR" << cameraMatrixR;
		fs << "cameraDistcoeffR" << distCoeffR;
		fs << "parameterR" << R;
		fs << "parameterT" << T;
		cout << "cameraMatrixL=:" << cameraMatrixL << endl << "cameraDistcoeffL=:" << distCoeffL << endl << "cameraMatrixR=:" << cameraMatrixR << endl << "cameraDistcoeffR=:" << distCoeffR << endl<< "R=" << R << endl << "T=" << T << endl;
		fs.release();
	}
	else
		cout << "Error: can not save the calibration parameters\n";
}


//��ʾrectify�Ľ����������ͼ
void showRecertifiedImage(Mat canvas, Mat rectifyImageL, Mat rectifyImageR)
{
	int w, h;
	w = imageSize.width/2;
	h = imageSize.height/2;

	/*��ͼ�񻭵�������*/
	Mat canvasPart = canvas(Rect(0, 0, w, h));								//�õ�������һ����
	resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);		//��ͼ�����ŵ���canvasPartһ����С
	Rect vroiL(validROIL.x/2, validROIL.y/2,				//��ñ���ȡ������	
		validROIL.width/2, validROIL.height/2);
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);						//����һ������

	cout << "Painted ImageL" << endl;

	/*��ͼ�񻭵�������*/
	canvasPart = canvas(Rect(w, 0, w, h));										//��û�������һ����
	resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(validROIR.x/2, validROIR.y/2,
		validROIR.width/2, validROIR.height/2);
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	cout << "Painted ImageR" << endl;

	/*���϶�Ӧ������*/
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
}

void readChessboardFrame(Mat grayImageL, Mat grayImageR)
{
	int goodFrameCount = 0;
	Mat rgbImageL, rgbImageR;
	vector<Point2f> cornerL;								//��������ĳһ��Ƭ�ǵ����꼯��
	vector<Point2f> cornerR;								//�ұ������ĳһ��Ƭ�ǵ����꼯��
	const Size boardSize = Size(boardWidth, boardHeight);	//
	while (goodFrameCount < frameNumber)
	{
		char filename[100];
		/*��ȡ��ߵ�ͼ��*/
		sprintf_s(filename, "left%04d.jpg", goodFrameCount + 1);
		rgbImageL = imread(filename, CV_LOAD_IMAGE_COLOR);
		cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);

		/*��ȡ�ұߵ�ͼ��*/
		sprintf_s(filename, "right%04d.jpg", goodFrameCount + 1);
		rgbImageR = imread(filename, CV_LOAD_IMAGE_COLOR);
		cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);

		bool isFindL, isFindR;

		isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);
		isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);
		if (isFindL == true && isFindR == true)	 //�������ͼ���ҵ������еĽǵ� ��˵��������ͼ���ǿ��е�
		{

			//Size(5,5)�������ڵ�һ���С
			//Size(-1,-1)������һ��ߴ�
			//TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)������ֹ����

			cornerSubPix(grayImageL, cornerL, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);
			imshow("chessboardL", rgbImageL);
			imagePointL.push_back(cornerL);

			cornerSubPix(grayImageR, cornerR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(rgbImageR, boardSize, cornerR, isFindR);
			imshow("chessboardR", rgbImageR);
			imagePointR.push_back(cornerR);

			goodFrameCount++;
			cout << "The image is good" << endl;
		}
		else
		{
			cout << "The image is bad please try again" << endl;
		}

		if (waitKey(10) == 'q')
		{
			break;
		}
	}
}

void calMapMatrix(void)
{
	/*
		�궨����ͷ
		��������������ֱ𶼾����˵�Ŀ�궨
		�����ڴ˴�ѡ��flag = CALIB_USE_INTRINSIC_GUESS
	*/
	double rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
		cameraMatrixL, distCoeffL,
		cameraMatrixR, distCoeffR,
		imageSize, R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));

	cout << "Stereo Calibration done with RMS error = " << rms << endl;

	/*
		����У����ʱ����Ҫ����ͼ���沢���ж�׼ ��ʹ������ƥ����ӵĿɿ�
		ʹ������ͼ����ķ������ǰ���������ͷ��ͼ��ͶӰ��һ�������������ϣ�����ÿ��ͼ��ӱ�ͼ��ƽ��ͶӰ������ͼ��ƽ�涼��Ҫһ����ת����R
		stereoRectify �����������ľ��Ǵ�ͼ��ƽ��ͶӰ����������ƽ�����ת����Rl,Rr�� Rl,Rr��Ϊ�������ƽ���ж�׼��У����ת����
		���������Rl��ת�����������Rr��ת֮������ͼ����Ѿ����沢���ж�׼�ˡ�
		����Pl,PrΪ���������ͶӰ�����������ǽ�3D�������ת����ͼ���2D�������:P*[X Y Z 1]' =[x y w]
		Q����Ϊ��ͶӰ���󣬼�����Q���԰�2άƽ��(ͼ��ƽ��)�ϵĵ�ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ���ʱ��
	*/
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
	/*
		����stereoRectify ���������R �� P ������ͼ���ӳ��� mapx,mapy
		mapx,mapy������ӳ�����������Ը�remap()�������ã���У��ͼ��ʹ������ͼ���沢���ж�׼
		ininUndistortRectifyMap()�Ĳ���newCameraMatrix����У����������������openCV���棬У����ļ��������Mrect�Ǹ�ͶӰ����Pһ�𷵻صġ�
		�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
	*/
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
}


int main(int argc, char* argv[])
{
	/*��������ͷ��������*/
	inputCameraParam();

	Mat img;

	namedWindow("ImageL");
	namedWindow("ImageR");
	cout << "��Q�˳� ..." << endl;

	Mat grayImageL(imageSize,CV_8UC1), grayImageR(imageSize, CV_8UC1);
	
	/*�������̸�ͼƬ*/
	readChessboardFrame(grayImageL,grayImageR);

	/*
		����ʵ�ʵ�У�������ά����
		����ʵ�ʱ궨���ӵĴ�С������
	*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	cout << "cal real successful" << endl;

	calMapMatrix();

	Mat rectifyImageL, rectifyImageR;
	cvtColor(grayImageL, rectifyImageL, CV_GRAY2BGR);
	cvtColor(grayImageR, rectifyImageR, CV_GRAY2BGR);

	/*
		����remap֮�����������ͼ���Ѿ����沢���ж�׼��
	*/
	remap(rectifyImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(rectifyImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	imshow("ImageL", rectifyImageL);
	imshow("ImageR", rectifyImageR);

	/*���沢�������*/
	outputCameraParam();

	/*
		��У�������ʾ����
		����������ͼ����ʾ��ͬһ��������
		����ֻ��ʾ�����һ��ͼ���У���������û�а����е�ͼ����ʾ����
	*/
	Mat canvas(imageSize.height/2, imageSize.width, CV_8UC3);
	showRecertifiedImage(canvas, rectifyImageL, rectifyImageR);

	imshow("rectified", canvas);

	cout << "wait key" << endl;
	waitKey(0);
	return 0;
}
