// ����У�����������ͼ��任�������ƽ�棬���У�����ͼƬ

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
	char* leftimg_directory = "..//..//..//imgs//leftImgs//";	// ������ͷͼ��·��
	char* rightimg_directory = "..//..//..//imgs//rightImgs//";	// ������ͷͼ��·��
	char* leftimg_filename = "left";							// ������ͷͼ����
	char* rightimg_filename = "right";							// ������ͷͼ����
	char* extension = "jpg";									// ͼ���ʽ
	char* calib_file = "..//..//..//parms//calibParms.xml";		// ����У�������ļ�
	char* leftout_directory = "..//..//..//result//left//";		// ������ͷͼ��У�����·��
	char* rightout_directory = "..//..//..//result//right//";	// ������ͷͼ��У�����·��
	char* leftout_filename = "leftOut";							// ������ͷͼ��У������ļ���
	char* rightout_filename = "rightOut";						// ������ͷͼ��У������ļ���
	int num_imgs = 13;
	Size imgSize = Size(640,480);

	Mat R1, R2, P1, P2, Q;
	Mat cameraMatrixL, cameraDistcoeffL, cameraMatrixR, cameraDistcoeffR, R, T;

	cv::FileStorage fs(calib_file, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("�򿪲����ļ�ʧ��!\n");
		return -1;
	}
	fs["cameraMatrixL"] >> cameraMatrixL;
	fs["cameraDistcoeffL"] >> cameraDistcoeffL;
	fs["cameraMatrixR"] >> cameraMatrixR;
	fs["cameraDistcoeffR"] >> cameraDistcoeffR;
	fs["R"] >> R;
	fs["T"] >> T;

	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs["Q"] >> Q;

	cv::Mat lmapx, lmapy, rmapx, rmapy;
	cv::initUndistortRectifyMap(cameraMatrixL, cameraDistcoeffL, R1, P1, imgSize, CV_32F, lmapx, lmapy);
	cv::initUndistortRectifyMap(cameraMatrixR, cameraDistcoeffR, R2, P2, imgSize, CV_32F, rmapx, rmapy);

	for (int k = 1; k <= num_imgs; k++)
	{
		char leftimg_file[100];
		char rightimg_file[100];
		sprintf(leftimg_file, "%s%s%d.%s", leftimg_directory, leftimg_filename, k, extension);
		sprintf(rightimg_file, "%s%s%d.%s", rightimg_directory, rightimg_filename, k, extension);
		Mat img1 = imread(leftimg_file, CV_LOAD_IMAGE_COLOR);
		Mat img2 = imread(rightimg_file, CV_LOAD_IMAGE_COLOR);
		if (img1.empty() || img2.empty())
		{
			printf("����ͼ�����\n");
			return -1;
		}

		cv::Mat imgU1, imgU2;

		cv::remap(img1, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
		cv::remap(img2, imgU2, rmapx, rmapy, cv::INTER_LINEAR);

		char leftout_file[100];
		sprintf(leftout_file, "%s%s%d.%s", leftout_directory, leftout_filename, k, extension);
		imwrite(leftout_file, imgU1);

		char rightout_file[100];
		sprintf(rightout_file, "%s%s%d.%s", rightout_directory, rightout_filename, k, extension);
		imwrite(rightout_file, imgU2);

		cout << k << ". recertify!" << endl;
	}
	
	return 0;
}