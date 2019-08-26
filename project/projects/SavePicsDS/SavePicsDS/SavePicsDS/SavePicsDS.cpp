// ʹ��DirectShow��˫Ŀ����ͷ������ͼƬ

#include "CameraDS.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char const *argv[])
{
	char* leftimg_directory = "..//..//..//imgs//leftImgs//";		// ����ͼ�񱣴�·��
	char* rightimg_directory = "..//..//..//imgs//rightImgs//";		// ����ͼ�񱣴�·��
	char* extension = "jpg";										// ����ͼ�񱣴��ʽ
	int img_width = 640, img_height = 480;							// ���ô�����ͷ��ȡ��ͼ���С


	if(CCameraDS::CameraCount() < 3) return 0;

    CCameraDS camera1, camera2;										// ����2������ͷ����

    // ����������ͷ
    if(!camera1.OpenCamera(0, false, img_width, img_height) ||
		(!camera2.OpenCamera(2, false, img_width, img_height)))
    {
        fprintf(stderr, "������ͷʧ�ܣ�\n");
        return -1;
    }

	IplImage *pFrame1, *pFrame2;
	Mat img1, img2;

	int iframe = 1;
	while (1)
	{
        // ��ȡһ֡
		pFrame1 = camera1.QueryFrame();
		pFrame2 = camera2.QueryFrame();

		img1 = Mat(pFrame1);
		img2 = Mat(pFrame2);
		imshow("IMG1", img1);
		imshow("IMG2", img2);
		if (waitKey(30) == 27) 
		{
			char filename1[200], filename2[200];
			sprintf_s(filename1, "%sleft%d.%s", leftimg_directory, iframe, extension);
			sprintf_s(filename2, "%sright%d.%s", rightimg_directory, iframe, extension);
			cout << "����� " << iframe << " ��ͼƬ" << endl;
			imwrite(filename1, img1);
			imwrite(filename2, img2);
			iframe++;
		}
	}
	camera1.CloseCamera(); // �ɲ����ô˺�����CCameraDS����ʱ���Զ��ر�����ͷ
    camera2.CloseCamera();
	return 0;
}