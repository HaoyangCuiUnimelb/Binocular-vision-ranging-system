// ������������ͷ�Ĳ�������������궨����������ļ�

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

int load_image_points(vector<vector<Point3f>> &object_points, vector<vector<Point2f>> &left_img_points, vector<vector<Point2f>> &right_img_points, int board_width, int board_height, int num_imgs, float square_size, char* leftimg_dir, char* rightimg_dir, char* leftimg_filename, char* rightimg_filename);

int main(int argc, char const *argv[])
{
	char* leftcalib_file = "..//..//..//parms//leftParms.xml";	// ������ͷ�����ļ�·�����ļ���
	char* rightcalib_file = "..//..//..//parms//rightParms.xml";// ������ͷ�����ļ�·�����ļ���
	char* leftimg_dir = "..//..//..//imgs//leftImgs//";			// ������ͷͼ��·��
	char* rightimg_dir= "..//..//..//imgs//rightImgs//";		// ������ͷͼ��·��
	char* leftimg_filename = "left";							// ������ͷͼ����
	char* rightimg_filename= "right";							// ������ͷͼ����
	char* out_file = "..//..//..//parms//calibParms.xml";		// ��������У�������ļ�·�����ļ���
	Size imgSize = Size( 640, 480 );							// ����ͼ���С
	int num_imgs = 13;											// ͼƬ��Ŀ

	FileStorage fsl(leftcalib_file, FileStorage::READ);
	FileStorage fsr(rightcalib_file, FileStorage::READ);
	if (!(fsl.isOpened() && fsr.isOpened()))
	{
		printf("��������ͷ���� %s �� %s ����\n", leftcalib_file, rightcalib_file);
		return -1;
	}
	vector< vector< Point3f > > object_points;
	vector<vector<Point2f>> left_img_points, right_img_points;
	int err = load_image_points(object_points, left_img_points, right_img_points, fsl["board_width"], fsl["board_height"], num_imgs, fsl["square_size"], leftimg_dir, rightimg_dir, leftimg_filename, rightimg_filename);
	if (err == -1)
	{
		return -1;
	}

	printf("��ʼ�궨...\n");

	Mat cameraMatrixL, cameraDistcoeffL, cameraMatrixR, cameraDistcoeffR, R, T, E, F;
	fsl["cameraMatrix"] >> cameraMatrixL;
	fsl["cameraDistcoeff"] >> cameraDistcoeffL;
	fsr["cameraMatrix"] >> cameraMatrixR;
	fsr["cameraDistcoeff"] >> cameraDistcoeffR;
	int flag = 0;
	flag |= CV_CALIB_FIX_INTRINSIC;

	cout << "�������..." << endl;
	stereoCalibrate(object_points, left_img_points, right_img_points, cameraMatrixL, cameraDistcoeffL, cameraMatrixR, cameraDistcoeffR, imgSize, R, T, E, F);

	cv::FileStorage fso(out_file, cv::FileStorage::WRITE);
	fso << "cameraMatrixL" << cameraMatrixL;
	fso << "cameraDistcoeffL" << cameraDistcoeffL;
	fso << "cameraMatrixR" << cameraMatrixR;
	fso << "cameraDistcoeffR" << cameraDistcoeffR;
	fso << "R" << R;
	fso << "T" << T;
	fso << "E" << E;
	fso << "F" << F;

	printf("�궨��ɣ�\n");

	printf("��ʼУ��...\n");

	cv::Mat R1, R2, P1, P2, Q;
	stereoRectify(cameraMatrixL, cameraDistcoeffL, cameraMatrixR, cameraDistcoeffR, imgSize, R, T, R1, R2, P1, P2, Q);

	fso << "R1" << R1;
	fso << "R2" << R2;
	fso << "P1" << P1;
	fso << "P2" << P2;
	fso << "Q" << Q;

	printf("У����ɣ�\n");

	return 0;
}


int load_image_points(vector<vector<Point3f>> &object_points, vector<vector<Point2f>> &left_img_points, vector<vector<Point2f>> &right_img_points, int board_width, int board_height, int num_imgs, float square_size, char* leftimg_dir, char* rightimg_dir, char* leftimg_filename, char* rightimg_filename) 
{
	vector< vector< Point2f > > imagePoints1, imagePoints2;
	vector< Point2f > corners1, corners2;
	Mat img1, img2, gray1, gray2;
	Size board_size = Size(board_width, board_height);
	int board_n = board_width * board_height;

	for (int i = 1; i <= num_imgs; i++)
	{
		char left_img[100], right_img[100];
		sprintf(left_img, "%s%s%d.jpg", leftimg_dir, leftimg_filename, i);
		sprintf(right_img, "%s%s%d.jpg", rightimg_dir, rightimg_filename, i);
		img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
		img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
		if (img1.empty() || img2.empty())
		{
			printf("��ȡͼ�� %s �� %s ʧ�ܣ�\n", left_img, right_img);
			return -1;
		}
		cvtColor(img1, gray1, CV_BGR2GRAY);
		cvtColor(img2, gray2, CV_BGR2GRAY);

		bool found1 = false, found2 = false;

		found1 = cv::findChessboardCorners(img1, board_size, corners1,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		found2 = cv::findChessboardCorners(img2, board_size, corners2,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if (found1)
		{
			cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(gray1, board_size, corners1, found1);
		}
		if (found2)
		{
			cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(gray2, board_size, corners2, found2);
		}

		vector< Point3f > obj;
		for (int i = 0; i < board_height; i++)
			for (int j = 0; j < board_width; j++)
				obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

		if (found1 && found2)
		{
			cout << "�� " << i << " ��ͼƬ��⵽�ǵ�!" << endl;
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);
			object_points.push_back(obj);
		}
	}
	for (int i = 0; i < imagePoints1.size(); i++)
	{
		vector< Point2f > v1, v2;
		for (int j = 0; j < imagePoints1[i].size(); j++)
		{
			v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
			v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
		}
		left_img_points.push_back(v1);
		right_img_points.push_back(v2);
	}
	return 0;
}
