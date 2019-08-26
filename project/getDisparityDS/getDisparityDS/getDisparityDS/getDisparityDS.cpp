#include "CameraDS.h"
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

using namespace cv;
using namespace std;

Size	imageSize;
int		imageWidth = 640;
int		imageHeight = 480;
bool	data_Loaded;			// �Ƿ����붨�����
Mat		Mat_Remap_X_L;			// ����ͼ����У����������ӳ����� X
Mat		Mat_Remap_Y_L;			// ����ͼ����У����������ӳ����� Y
Mat		Mat_Remap_X_R;			// ����ͼ����У����������ӳ����� X
Mat		Mat_Remap_Y_R;			// ����ͼ����У����������ӳ����� Y
Mat		Mat_Mask_Roi;			// ����ͼУ�������Ч����
Rect	Roi_L;					// ����ͼУ�������Ч�������
Rect	Roi_R;					// ����ͼУ�������Ч�������
int		numberOfDisparies;		// �Ӳ�仯��Χ
cv::StereoBM	BM;				// ����ƥ�� BM ����
cv::StereoSGBM	SGBM;			// ����ƥ�� SGBM ����
Mat		disp, disp8u, disparityImage;

Mat cameraMatrixL = (Mat_<double>(3, 3));
Mat distCoeffL = (Mat_<double>(5, 1));
Mat cameraMatrixR = (Mat_<double>(3, 3));
Mat distCoeffR = (Mat_<double>(5, 1));
Mat R = (Mat_<double>(3, 3));	// R ��ת���� 
Mat T = (Mat_<double>(3, 1));	// T ƽ��ʸ��
Mat E, F;						// E �������� F ��������
Mat Mat_Q;						// Q ����
Mat Rl, Rr, Pl, Pr;				// У����ת���� Rl Rr��ͶӰ���� Pl Pr

// ��������ͷ�������������У��
int loadAndCalData();

// ���� BM ����
void updatebm();

// ���� sgBM ����
void updatesgbm();

// BM ������ƥ��
int  bmMatch(cv::Mat& frameLeft, cv::Mat& frameRight, cv::Mat& disparity, cv::Mat& imageLeft, cv::Mat& imageRight);

// SGBM ������ƥ��
int  sgbmMatch(cv::Mat& frameLeft, cv::Mat& frameRight, cv::Mat& disparity, cv::Mat& imageLeft, cv::Mat& imageRight);

// ��ȡ�Ӳ�ͼ
int getDisparityImage(cv::Mat& disparity, cv::Mat& disparityImage, bool isColor);


int main(int argc, char** argv)
{
	if(CCameraDS::CameraCount() < 3)
		return 0;

	// ����2������ͷ����
	CCameraDS camera1;
	CCameraDS camera2;

	//����������ͷ
	if(!camera1.OpenCamera(0, false, 640,480) || (!camera2.OpenCamera(2, false, 640,480)))
	{
		fprintf(stderr, "Can not open camera.");
		return -1;
	}

	namedWindow("left", 1);
	namedWindow("right", 1);
	namedWindow("disparity", 1);
	loadAndCalData();
	cout << data_Loaded << endl;

	Mat frame1,frame2;//������ͼ

	while (true)
	{
		//��ȡһ֡
		frame1 = Mat(camera1.QueryFrame());
		frame2 = Mat(camera2.QueryFrame());

		if (frame1.empty())	break;
		if (frame2.empty())	break;

		Mat imageLeft, imageRight;//˫Ŀƥ����������ͼ
		//����ƥ���������Ӳ�
		updatebm();
		bmMatch(frame1, frame2, disp, imageLeft, imageRight);
		//updatesgbm();
		//sgbmMatch(frame1, frame2, disp, imageLeft, imageRight);
		getDisparityImage(disp, disparityImage, false);

		imshow("disparity", disparityImage);//��ʾ�Ӳ�ͼ
		imshow("left", imageLeft);//��ʾ˫Ŀƥ����������ͼ
		imshow("right", imageRight);


		if(27 == waitKey(1)) break;
	}
	return 0;
}


int loadAndCalData()
{
	// ��������ͷ�������
	FileStorage fs("stereoParams.xml", FileStorage::READ);
	cout << fs.isOpened() << endl;

	if (!fs.isOpened())
	{
		data_Loaded = false;
		return 0;
	}

	cv::FileNodeIterator it = fs["imageSize"].begin();
	it >> imageSize.width >> imageSize.height;
	fs["cameraMatrixL"] >> cameraMatrixL;
	fs["cameraDistcoeffL"] >> distCoeffL;
	fs["cameraMatrixR"] >> cameraMatrixR;
	fs["cameraDistcoeffR"] >> distCoeffR;
	fs["R"] >> R;
	fs["T"] >> T;

	fs.release();
	data_Loaded = true;

	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Mat_Q,
		CALIB_ZERO_DISPARITY,-1,imageSize,&Roi_L,&Roi_R);

	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_16SC2, Mat_Remap_X_L, Mat_Remap_Y_L);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_16SC2, Mat_Remap_X_R, Mat_Remap_Y_R);

	Mat_Q.at<double>(3, 2) = -Mat_Q.at<double>(3, 2);

	Mat_Mask_Roi = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
	cv::rectangle(Mat_Mask_Roi, Roi_L, cv::Scalar(255), -1);

	return 1;
}

void updatebm()
{
	BM.state->roi1 = Roi_L;
	BM.state->roi2 = Roi_R;
	BM.state->minDisparity = 0;
	BM.state->numberOfDisparities = 64;
	BM.state->SADWindowSize = 19;
	BM.state->textureThreshold = 10;
	BM.state->disp12MaxDiff = -1;
	BM.state->preFilterCap = 31;
	BM.state->uniquenessRatio = 25;
	BM.state->speckleRange = 32;
	BM.state->speckleWindowSize = 100;
}

void updatesgbm()
{
	SGBM.preFilterCap = 63;
	SGBM.SADWindowSize = 7;
	SGBM.P1 = 8 * 3 * SGBM.SADWindowSize * SGBM.SADWindowSize ;
	SGBM.P2 = 32 * 3 * SGBM.SADWindowSize * SGBM.SADWindowSize ;
	SGBM.minDisparity = 0;
	SGBM.numberOfDisparities = 64;
	SGBM.uniquenessRatio = 25;
	SGBM.speckleWindowSize = 100;
	SGBM.speckleRange =32;
	SGBM.fullDP = false;
	SGBM.disp12MaxDiff = -1;
}

int  bmMatch(cv::Mat& frameLeft, cv::Mat& frameRight, cv::Mat& disparity, cv::Mat& imageLeft, cv::Mat& imageRight)
{
	// ������
	if (frameLeft.empty() || frameRight.empty())
	{
		disparity = cv::Scalar(0);
		return 0;
	}
	if (imageWidth == 0 || imageHeight == 0)
	{
		return 0;
	}

	// ת��Ϊ�Ҷ�ͼ
	cv::Mat img1proc, img2proc;
	cvtColor(frameLeft, img1proc, CV_BGR2GRAY);
	cvtColor(frameRight, img2proc, CV_BGR2GRAY);

	// У��ͼ��ʹ������ͼ�ж���    
	cv::Mat img1remap, img2remap;

	if (data_Loaded)
	{
		// �������Ӳ����Ļ������У��
		remap(img1proc, img1remap, Mat_Remap_X_L, Mat_Remap_Y_L, cv::INTER_LINEAR);     
		remap(img2proc, img2remap, Mat_Remap_X_R, Mat_Remap_Y_R, cv::INTER_LINEAR);
	}
	else
	{
		img1remap = img1proc;
		img2remap = img2proc;
	}

	// ��������ͼ����߽��б߽����أ��Ի�ȡ��ԭʼ��ͼ��ͬ��С����Ч�Ӳ�����
	cv::Mat img1border, img2border;
	if (numberOfDisparies != BM.state->numberOfDisparities)
		numberOfDisparies = BM.state->numberOfDisparities;
	copyMakeBorder(img1remap, img1border, 0, 0, BM.state->numberOfDisparities, 0, IPL_BORDER_REPLICATE);
	copyMakeBorder(img2remap, img2border, 0, 0, BM.state->numberOfDisparities, 0, IPL_BORDER_REPLICATE);

	// �����Ӳ�
	cv::Mat dispBorder;
	BM(img1border, img2border, dispBorder);

	// ��ȡ��ԭʼ�����Ӧ���Ӳ�������ȥ�ӿ�Ĳ��֣�
	cv::Mat disp;
	disp = dispBorder.colRange(BM.state->numberOfDisparities, img1border.cols);
	disp.copyTo(disparity, Mat_Mask_Roi);

	// ���������ͼ��
	if (data_Loaded)
	{
		remap(frameLeft, imageLeft, Mat_Remap_X_L, Mat_Remap_Y_L, cv::INTER_LINEAR);
		rectangle(imageLeft, Roi_L, CV_RGB(0, 0, 255), 3);
	}
	else
		frameLeft.copyTo(imageLeft);

	if (data_Loaded)
		remap(frameRight, imageRight, Mat_Remap_X_R, Mat_Remap_Y_R, cv::INTER_LINEAR);
	else
		frameRight.copyTo(imageRight);
	rectangle(imageRight, Roi_R, CV_RGB(0, 0, 255), 3);

	return 1;
}


int sgbmMatch(cv::Mat& frameLeft, cv::Mat& frameRight, cv::Mat& disparity, cv::Mat& imageLeft, cv::Mat& imageRight)
{
	// ������
	if (frameLeft.empty() || frameRight.empty())
	{
		disparity = cv::Scalar(0);
		return 0;
	}
	if (imageWidth == 0 || imageHeight == 0)
	{
		return 0;
	}

	// ����ͼ��
	cv::Mat img1proc, img2proc;
	frameLeft.copyTo(img1proc);
	frameRight.copyTo(img2proc);

	// У��ͼ��ʹ������ͼ�ж���	
	cv::Mat img1remap, img2remap;
	if (data_Loaded)
	{
		// �������Ӳ����Ļ������У��
		remap(img1proc, img1remap, Mat_Remap_X_L, Mat_Remap_Y_L, cv::INTER_LINEAR);		
		remap(img2proc, img2remap, Mat_Remap_X_R, Mat_Remap_Y_R, cv::INTER_LINEAR);
	} 
	else
	{
		img1remap = img1proc;
		img2remap = img2proc;
	}

	// ��������ͼ����߽��б߽����أ��Ի�ȡ��ԭʼ��ͼ��ͬ��С����Ч�Ӳ�����
	cv::Mat img1border, img2border;
	if (numberOfDisparies != SGBM.numberOfDisparities)
		numberOfDisparies = SGBM.numberOfDisparities;
	copyMakeBorder(img1remap, img1border, 0, 0, SGBM.numberOfDisparities, 0, IPL_BORDER_REPLICATE);
	copyMakeBorder(img2remap, img2border, 0, 0, SGBM.numberOfDisparities, 0, IPL_BORDER_REPLICATE);

	// �����Ӳ�
	cv::Mat dispBorder;
	SGBM(img1border, img2border, dispBorder);

	// ��ȡ��ԭʼ�����Ӧ���Ӳ�������ȥ�ӿ�Ĳ��֣�
	cv::Mat disp;
	disp = dispBorder.colRange(SGBM.numberOfDisparities, img1border.cols);	
	disp.copyTo(disparity, Mat_Mask_Roi);

	// ���������ͼ��
	imageLeft = img1remap.clone();
	imageRight = img2remap.clone();
	rectangle(imageLeft, Roi_L, CV_RGB(0,255,0), 3);
	rectangle(imageRight, Roi_R, CV_RGB(0,255,0), 3);

	return 1;
}


int getDisparityImage(cv::Mat& disparity, cv::Mat& disparityImage, bool isColor)
{
	// ��ԭʼ�Ӳ����ݵ�λ��ת��Ϊ 8 λ
	cv::Mat disp8u;
	if (disparity.depth() != CV_8U)
	{
		if (disparity.depth() == CV_8S)
		{
			disparity.convertTo(disp8u, CV_8U);
		}
		else
		{
			disparity.convertTo(disp8u, CV_8U, 255 / (numberOfDisparies*16.));
		}
	}
	else
	{
		disp8u = disparity;
	}

	// ת��Ϊα��ɫͼ�� �� �Ҷ�ͼ��
	if (isColor)
	{
		if (disparityImage.empty() || disparityImage.type() != CV_8UC3 || disparityImage.size() != disparity.size())
		{
			disparityImage = cv::Mat::zeros(disparity.rows, disparity.cols, CV_8UC3);
		}

		for (int y = 0; y<disparity.rows; y++)
		{
			for (int x = 0; x<disparity.cols; x++)
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

				disparityImage.at<cv::Vec3b>(y, x) = cv::Vec3b(r, g, b);
			}
		}
	}
	else
	{
		disp8u.copyTo(disparityImage);
	}

	return 1;
}