// 使用DirectShow打开双目摄像头，保存图片

#include "CameraDS.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char const *argv[])
{
	char* leftimg_directory = "..//..//..//imgs//leftImgs//";		// 设置图像保存路径
	char* rightimg_directory = "..//..//..//imgs//rightImgs//";		// 设置图像保存路径
	char* extension = "jpg";										// 设置图像保存格式
	int img_width = 640, img_height = 480;							// 设置从摄像头获取的图像大小


	if(CCameraDS::CameraCount() < 3) return 0;

    CCameraDS camera1, camera2;										// 创建2个摄像头对象

    // 打开两个摄像头
    if(!camera1.OpenCamera(0, false, img_width, img_height) ||
		(!camera2.OpenCamera(2, false, img_width, img_height)))
    {
        fprintf(stderr, "打开摄像头失败！\n");
        return -1;
    }

	IplImage *pFrame1, *pFrame2;
	Mat img1, img2;

	int iframe = 1;
	while (1)
	{
        // 获取一帧
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
			cout << "保存第 " << iframe << " 对图片" << endl;
			imwrite(filename1, img1);
			imwrite(filename2, img2);
			iframe++;
		}
	}
	camera1.CloseCamera(); // 可不调用此函数，CCameraDS析构时会自动关闭摄像头
    camera2.CloseCamera();
	return 0;
}