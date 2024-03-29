// stereoCalibration.cpp : 定义控制台应用程序的入口点。
//
//在进行双目摄像头的标定之前，事先分别对两个摄像头进行单目视觉的标定 
//确定两个摄像头的内参矩阵，然后再进行立体标定

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

const int imageWidth = 640;								//摄像头的分辨率
const int imageHeight = 480;
const int boardWidth = 9;								//横向的角点数目
const int boardHeight = 6;								//纵向的角点数据
const int boardCorner = boardWidth * boardHeight;		//总的角点数据
const int frameNumber = 12;								//相机标定时需要采用的图像帧数
const int squareSize = 25;								//标定板黑白格子的大小 单位mm

Size imageSize = Size(imageWidth, imageHeight);

Mat R, T, E, F;											//R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵
vector<Mat> rvecs;									    //旋转向量
vector<Mat> tvecs;										//平移向量
vector<vector<Point2f>> imagePointL;				    //左边摄像机所有照片角点的坐标集合
vector<vector<Point2f>> imagePointR;					//右边摄像机所有照片角点的坐标集合
vector<vector<Point3f>> objRealPoint;					//各副图像的角点的实际物理坐标集合

Mat Rl, Rr, Pl, Pr, Q;									//校正旋转矩阵R，投影矩阵P 重投影矩阵Q 	
Mat mapLx, mapLy, mapRx, mapRy;							//映射表
Rect validROIL, validROIR;								//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域

Mat cameraMatrixL = (Mat_<double>(3, 3));
Mat distCoeffL = (Mat_<double>(5, 1));
Mat cameraMatrixR = (Mat_<double>(3, 3));
Mat distCoeffR = (Mat_<double>(5, 1));

// /*
// 事先标定好的左相机的内参矩阵
// fx 0 cx
// 0 fy cy
// 0 0  1
// */
// Mat cameraMatrixL = (Mat_<double>(3, 3) << 532.782, 0,       532.904,
// 										  0,       342.505, 233.876,
// 										  0,       0,       1);
// Mat distCoeffL = (Mat_<double>(5, 1) << -0.28095, 0.0255745, 0.00122226, -0.000137736, 0.162946);
// /*
// 事先标定好的右相机的内参矩阵
// fx 0 cx
// 0 fy cy
// 0 0  1
// */
// Mat cameraMatrixR = (Mat_<double>(3, 3) << 532.782, 0,       532.904,
// 											0,      342.505, 233.876,
// 											0,		0,		 1);
// Mat distCoeffR = (Mat_<double>(5, 1) << -0.28095, 0.0255745, 0.00122226, -0.000137736, 0.162946);

/*计算标定板上模块的实际物理坐标*/
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
	/*读入数据*/
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
	/*保存数据*/
	/*输出数据*/
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


//显示rectify的结果：左右视图
void showRecertifiedImage(Mat canvas, Mat rectifyImageL, Mat rectifyImageR)
{
	int w, h;
	w = imageSize.width/2;
	h = imageSize.height/2;

	/*左图像画到画布上*/
	Mat canvasPart = canvas(Rect(0, 0, w, h));								//得到画布的一部分
	resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);		//把图像缩放到跟canvasPart一样大小
	Rect vroiL(validROIL.x/2, validROIL.y/2,				//获得被截取的区域	
		validROIL.width/2, validROIL.height/2);
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);						//画上一个矩形

	cout << "Painted ImageL" << endl;

	/*右图像画到画布上*/
	canvasPart = canvas(Rect(w, 0, w, h));										//获得画布的另一部分
	resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(validROIR.x/2, validROIR.y/2,
		validROIR.width/2, validROIR.height/2);
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	cout << "Painted ImageR" << endl;

	/*画上对应的线条*/
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
}

void readChessboardFrame(Mat grayImageL, Mat grayImageR)
{
	int goodFrameCount = 0;
	Mat rgbImageL, rgbImageR;
	vector<Point2f> cornerL;								//左边摄像机某一照片角点坐标集合
	vector<Point2f> cornerR;								//右边摄像机某一照片角点坐标集合
	const Size boardSize = Size(boardWidth, boardHeight);	//
	while (goodFrameCount < frameNumber)
	{
		char filename[100];
		/*读取左边的图像*/
		sprintf_s(filename, "left%04d.jpg", goodFrameCount + 1);
		rgbImageL = imread(filename, CV_LOAD_IMAGE_COLOR);
		cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);

		/*读取右边的图像*/
		sprintf_s(filename, "right%04d.jpg", goodFrameCount + 1);
		rgbImageR = imread(filename, CV_LOAD_IMAGE_COLOR);
		cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);

		bool isFindL, isFindR;

		isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);
		isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);
		if (isFindL == true && isFindR == true)	 //如果两幅图像都找到了所有的角点 则说明这两幅图像是可行的
		{

			//Size(5,5)搜索窗口的一半大小
			//Size(-1,-1)死区的一半尺寸
			//TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)迭代终止条件

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
		标定摄像头
		由于左右摄像机分别都经过了单目标定
		所以在此处选择flag = CALIB_USE_INTRINSIC_GUESS
	*/
	double rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
		cameraMatrixL, distCoeffL,
		cameraMatrixR, distCoeffR,
		imageSize, R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));

	cout << "Stereo Calibration done with RMS error = " << rms << endl;

	/*
		立体校正的时候需要两幅图像共面并且行对准 以使得立体匹配更加的可靠
		使得两幅图像共面的方法就是把两个摄像头的图像投影到一个公共成像面上，这样每幅图像从本图像平面投影到公共图像平面都需要一个旋转矩阵R
		stereoRectify 这个函数计算的就是从图像平面投影都公共成像平面的旋转矩阵Rl,Rr。 Rl,Rr即为左右相机平面行对准的校正旋转矩阵。
		左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面并且行对准了。
		其中Pl,Pr为两个相机的投影矩阵，其作用是将3D点的坐标转换到图像的2D点的坐标:P*[X Y Z 1]' =[x y w]
		Q矩阵为重投影矩阵，即矩阵Q可以把2维平面(图像平面)上的点投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的时差
	*/
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
	/*
		根据stereoRectify 计算出来的R 和 P 来计算图像的映射表 mapx,mapy
		mapx,mapy这两个映射表接下来可以给remap()函数调用，来校正图像，使得两幅图像共面并且行对准
		ininUndistortRectifyMap()的参数newCameraMatrix就是校正后的摄像机矩阵。在openCV里面，校正后的计算机矩阵Mrect是跟投影矩阵P一起返回的。
		所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
	*/
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
}


int main(int argc, char* argv[])
{
	/*读入摄像头参数矩阵*/
	inputCameraParam();

	Mat img;

	namedWindow("ImageL");
	namedWindow("ImageR");
	cout << "按Q退出 ..." << endl;

	Mat grayImageL(imageSize,CV_8UC1), grayImageR(imageSize, CV_8UC1);
	
	/*载入棋盘格图片*/
	readChessboardFrame(grayImageL,grayImageR);

	/*
		计算实际的校正点的三维坐标
		根据实际标定格子的大小来设置
	*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	cout << "cal real successful" << endl;

	calMapMatrix();

	Mat rectifyImageL, rectifyImageR;
	cvtColor(grayImageL, rectifyImageL, CV_GRAY2BGR);
	cvtColor(grayImageR, rectifyImageR, CV_GRAY2BGR);

	/*
		经过remap之后，左右相机的图像已经共面并且行对准了
	*/
	remap(rectifyImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(rectifyImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	imshow("ImageL", rectifyImageL);
	imshow("ImageR", rectifyImageR);

	/*保存并输出数据*/
	outputCameraParam();

	/*
		把校正结果显示出来
		把左右两幅图像显示到同一个画面上
		这里只显示了最后一副图像的校正结果。并没有把所有的图像都显示出来
	*/
	Mat canvas(imageSize.height/2, imageSize.width, CV_8UC3);
	showRecertifiedImage(canvas, rectifyImageL, rectifyImageR);

	imshow("rectified", canvas);

	cout << "wait key" << endl;
	waitKey(0);
	return 0;
}
