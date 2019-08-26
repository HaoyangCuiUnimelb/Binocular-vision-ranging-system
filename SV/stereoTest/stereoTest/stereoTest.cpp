#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

using namespace cv;
using namespace std;

int pic_info[4];				// ����ȡ������
int m_frameWidth = 640;			// ͼ�����
int m_frameHeight = 480;
bool m_Calib_Data_Loaded;		// �Ƿ�ɹ����붨�����
Mat m_Calib_Mat_Remap_X_L;      // ������ͼ����У����������ӳ�����X��Y
Mat m_Calib_Mat_Remap_Y_L;
Mat m_Calib_Mat_Remap_X_R;
Mat m_Calib_Mat_Remap_Y_R;
Mat m_Calib_Mat_Mask_Roi;       // ����ͼУ�������Ч�������
Rect m_Calib_Roi_L;             // ������ͼУ�������Ч�������
Rect m_Calib_Roi_R;
Size imageSize;					// �����ļ�����ı궨ͼ���С
int numberOfDisparies = 96;		// �Ӳ�仯��Χ
Ptr<StereoBM> bm = StereoBM::create(16, 9);				// ʵ����BMƥ�����
Mat imageLeft, imageRight;		// ����������������ͼ
Mat disp, disp8u, disparityImage;						// �Ӳ�ͼ��8λ�Ӳ�ͼ����ɫ�Ӳ�ͼ
Mat pointClouds;				// ����
Mat cameraMatrixL = (Mat_<double>(3, 3));				// ��������ڲξ��󡢻����������
Mat distCoeffL = (Mat_<double>(5, 1));
Mat cameraMatrixR = (Mat_<double>(3, 3));
Mat distCoeffR = (Mat_<double>(5, 1));
Mat Rod = (Mat_<double>(3, 1));							// Rod ��ת����
Mat R = (Mat_<double>(3, 3)), T = (Mat_<double>(3, 1));	// R ��תʸ�� T ƽ��ʸ��
Mat E, F;												// E �������� F ��������
Mat m_Calib_Mat_Q;										// Q ����
Mat Rl, Rr, Pl, Pr;										// R ������ת���� P ͶӰ����


int loadCalibData(void)
{
    // ��������ͷ�������
	try
	{
		FileStorage fs("calib_paras.xml", FileStorage::READ);
		cout << fs.isOpened() << endl;

		if (!fs.isOpened())
		{
			return (0);
		}

		cv::FileNodeIterator it = fs["imageSize"].begin();

		it >> imageSize.width >> imageSize.height;
		
		fs["cameraMatrixL"] >> cameraMatrixL;
		fs["cameraDistcoeffL"] >> distCoeffL;
		fs["cameraMatrixR"] >> cameraMatrixR;
		fs["cameraDistcoeffR"] >> distCoeffR;
		fs["Rod"] >> Rod;
		fs["T"] >> T;

		fs.release();
        m_Calib_Data_Loaded = true;
    }
    catch (std::exception& e)
    {
        m_Calib_Data_Loaded = false;
        return (0);
    }

    return 1;
}


int  calParams(void)
{
	if (!m_Calib_Data_Loaded)
	{
		return 0;
	}

	// ���궨��ͼ��ߴ��뵱ǰͼ��ߴ��Ƿ�һ��
	if (m_frameWidth != imageSize.width || m_frameHeight != imageSize.height)
	{
		return 0;
	}
	
	Rodrigues(Rod,R);//����ת����ת��Ϊ��ת����

	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, m_Calib_Mat_Q,
				  CALIB_ZERO_DISPARITY,-1,imageSize,&m_Calib_Roi_L,&m_Calib_Roi_R);

	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_16SC2, m_Calib_Mat_Remap_X_L, m_Calib_Mat_Remap_Y_L);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_16SC2, m_Calib_Mat_Remap_X_R, m_Calib_Mat_Remap_Y_R);

	m_Calib_Mat_Q.at<double>(3, 2) = -m_Calib_Mat_Q.at<double>(3, 2);

	m_Calib_Mat_Mask_Roi = cv::Mat::zeros(m_frameHeight, m_frameWidth, CV_8UC1);
	cv::rectangle(m_Calib_Mat_Mask_Roi, m_Calib_Roi_L, cv::Scalar(255), -1);

	return 1;
}


void updatebm()
{
	bm->setROI1(m_Calib_Roi_L);
	bm->setROI2(m_Calib_Roi_R);
	bm->setPreFilterCap(31);
	bm->setBlockSize(15);
	bm->setMinDisparity(0);
	bm->setNumDisparities(numberOfDisparies);
	bm->setTextureThreshold(16);
	bm->setUniquenessRatio(25);
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);
}


int  bmMatch(cv::Mat& frameLeft, cv::Mat& frameRight, cv::Mat& disparity, cv::Mat& imageLeft, cv::Mat& imageRight)
{
    // ������
    if (frameLeft.empty() || frameRight.empty())
    {
        disparity = cv::Scalar(0);
        return 0;
    }
    if (m_frameWidth != imageSize.width || m_frameHeight != imageSize.height)
    {
        return 0;
    }

    // ת��Ϊ�Ҷ�ͼ
    cv::Mat img1proc, img2proc;
    cvtColor(frameLeft, img1proc, CV_BGR2GRAY);
    cvtColor(frameRight, img2proc, CV_BGR2GRAY);

    // У��ͼ��ʹ������ͼ�ж���    
    cv::Mat img1remap, img2remap;

    if (m_Calib_Data_Loaded)
    {
        remap(img1proc, img1remap, m_Calib_Mat_Remap_X_L, m_Calib_Mat_Remap_Y_L, cv::INTER_LINEAR);     // �������Ӳ����Ļ������У��
        remap(img2proc, img2remap, m_Calib_Mat_Remap_X_R, m_Calib_Mat_Remap_Y_R, cv::INTER_LINEAR);
    }
    else
    {
        img1remap = img1proc;
        img2remap = img2proc;
    }

    // ��������ͼ����߽��б߽����أ��Ի�ȡ��ԭʼ��ͼ��ͬ��С����Ч�Ӳ�����
    cv::Mat img1border, img2border;

    copyMakeBorder(img1remap, img1border, 0, 0, numberOfDisparies, 0, IPL_BORDER_REPLICATE);
    copyMakeBorder(img2remap, img2border, 0, 0, numberOfDisparies, 0, IPL_BORDER_REPLICATE);

    // �����Ӳ�
    cv::Mat dispBorder;
    bm->compute(img1border, img2border, dispBorder);

    // ��ȡ��ԭʼ�����Ӧ���Ӳ�������ȥ�ӿ�Ĳ��֣�
    cv::Mat disp;
    disp = dispBorder.colRange(numberOfDisparies, img1border.cols);
    disp.copyTo(disparity, m_Calib_Mat_Mask_Roi);

    // ���������ͼ��
    if (m_Calib_Data_Loaded)
    {
        remap(frameLeft, imageLeft, m_Calib_Mat_Remap_X_L, m_Calib_Mat_Remap_Y_L, cv::INTER_LINEAR);
        rectangle(imageLeft, m_Calib_Roi_L, CV_RGB(0, 0, 255), 3);
    }
    else
        frameLeft.copyTo(imageLeft);

    if (m_Calib_Data_Loaded)
        remap(frameRight, imageRight, m_Calib_Mat_Remap_X_R, m_Calib_Mat_Remap_Y_R, cv::INTER_LINEAR);
    else
        frameRight.copyTo(imageRight);
    rectangle(imageRight, m_Calib_Roi_R, CV_RGB(0, 0, 255), 3);

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

int getPointClouds(cv::Mat& disparity, cv::Mat& pointClouds)
{
    if (disparity.empty())
    {
        return 0;
    }

    // ����������ά����
    reprojectImageTo3D(disparity, pointClouds, m_Calib_Mat_Q, true);

    pointClouds *= 1.6;

    for (int y = 0; y < pointClouds.rows; ++y)
    {
        for (int x = 0; x < pointClouds.cols; ++x)
        {
            cv::Point3f point = pointClouds.at<cv::Point3f>(y, x);
            point.y = -point.y;
            pointClouds.at<cv::Point3f>(y, x) = point;
        }
    }

    return 1;
}

int detectDistance(cv::Mat& pointCloud)
{
    if (pointCloud.empty())
    {
        return 0;
    }

    // ��ȡ���ͼ��
    vector<cv::Mat> xyzSet;
    split(pointCloud, xyzSet);
    cv::Mat depth;
    xyzSet[2].copyTo(depth);

    // ���������ֵ���ж�ֵ������
    double maxVal = 0, minVal = 0;
    cv::Mat depthThresh = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1);
    cv::minMaxLoc(depth, &minVal, &maxVal);
    double thrVal = minVal * 1.5;
    threshold(depth, depthThresh, thrVal, 255, CV_THRESH_BINARY_INV);
    depthThresh.convertTo(depthThresh, CV_8UC1);

    double  distance = depthThresh.at<float>(pic_info[0], pic_info[1]);
    cout << "distance:" << distance << endl;
	return 1;
}


static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        pic_info[0] = x;
        pic_info[1] = y;
        cout << "x:" << pic_info[0] << endl <<"y:" << pic_info[1] << endl;
		detectDistance(pointClouds);
    }
}


// �ӵ�֡�л�ȡ��������ͼ��
int get2ImagesFrom1Frame(Mat frame, Mat frame1, Mat frame2)
{
	int cols = frame.cols;
	int rows = frame.rows;
	for (int i = 0; i < rows; i++)
	{
		uchar* data = frame.ptr<uchar>(i);
		uchar* data1 = frame1.ptr<uchar>(i);
		uchar* data2 = frame2.ptr<uchar>(i);

		for (int j = 0; j < cols / 2 * 3; j++)
		{
			data1[j] = data[j];
		}
		for (int j = 0; j < cols / 2 * 3; j++)
		{
			data2[j] = data[cols / 2 * 3 + j];
		}
	}
	return 1;
}


int main(int argc, char** argv)
{
	//// ��ȡ����ͷ
	//VideoCapture cap(0); 
	//VideoCapture cap1(1);

	// if (!cap.isOpened())
	// {
	//     cout << "error happened while open cam 1"<<endl;
	//     return -1;
	// }
	// if (!cap1.isOpened())
	// {
	//     cout << "error happened while open cam 2" << endl;
	//     return -1;
	// }

	///
	VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_WIDTH, m_frameWidth * 2);
	cap.set(CAP_PROP_FRAME_HEIGHT, m_frameHeight);

	if (!cap.isOpened())
	{
		cout << "camera open failed!" << endl;
		return 0;
	}

	m_frameWidth = cap.get(CAP_PROP_FRAME_WIDTH) / 2;
	m_frameHeight = cap.get(CAP_PROP_FRAME_HEIGHT);

	cout << "width:  " << m_frameWidth << endl;
	cout << "height: " << m_frameHeight << endl;

	Mat frame(m_frameHeight, m_frameWidth * 2, CV_8UC3);
	Mat frame1(m_frameHeight, m_frameWidth, CV_8UC3), frame2(m_frameHeight, m_frameWidth, CV_8UC3);
	///

	namedWindow("left", 1);
	namedWindow("right", 1);
	namedWindow("disparitycolor", 1);

	setMouseCallback("disparitycolor", onMouse, 0);

	if (0 == loadCalibData())
	{
		cout << "load CalibData failed!" << endl;
		return 0;
	}


	if (0 == calParams())
	{
		cout << "calculate parameters failed!" << endl;
		return 0;
	}


	while (true)
	{

		//Mat frame1;
		//Mat frame2;
		//cap.read(frame);
		//cap1.read(frame1);
		//if (frame1.empty())         break;
		//if (frame2.empty())         break;

		///
		cap >> frame;

		if (frame.empty())
		{
			cout << "image load failed!" << endl;
			return 0;
		}

		if (0 == get2ImagesFrom1Frame(frame, frame1, frame2))
		{
			cout << "get 2 images from 1 frame failed!" << endl;
			return 0;

		}
		///
		Mat image1;
		Mat image2;

		frame1.copyTo(image1);
		frame2.copyTo(image2);

		updatebm();

		if (0 == bmMatch(image1, image2, disp, imageLeft, imageRight))
		{
			cout << "stereo match failed!" << endl;
			return 0;
		}
			
		imshow("left", imageLeft);
		imshow("right", imageRight);

		if (0 == getDisparityImage(disp, disparityImage, true))
		{
			cout << "get disparityImage failed!" << endl;
			return 0;
		}
			
		if (0 == getPointClouds(disp, pointClouds))
		{
			cout << "get pointsclouds failed!" << endl;
			return 0;
		}

		imshow("disparitycolor", disparityImage);

		waitKey(10);
	}
	return 1;
}