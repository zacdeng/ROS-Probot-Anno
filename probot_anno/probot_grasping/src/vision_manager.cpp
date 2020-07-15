/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/
#include <opencv/cv.h>
#include "probot_grasping/vision_manager.h"

VisionManager::VisionManager(float length, float breadth)
{
	this->table_length = length;
	this->table_breadth = breadth;
}

void VisionManager::get2DLocation_Blue(cv::Mat img, float &x, float &y)
{
	this->curr_img = img;
	img_centre_x_ = img.rows / 2;
	img_centre_y_ = img.cols / 2;

	cv::Rect tablePos;

	detectTable(tablePos);  // 检测桌面轮廓

	detect2DObject_Blue(x, y, tablePos);  // 检测绿色物体位姿
	convertToMM(x, y); // 计算物体的中心位置
}

void VisionManager::get2DLocation_Green(cv::Mat img, float &x, float &y)
{
	this->curr_img = img;
	img_centre_x_ = img.rows / 2;
	img_centre_y_ = img.cols / 2;

	cv::Rect tablePos;

	detectTable(tablePos);  // 检测桌面轮廓

	detect2DObject_Green(x, y, tablePos);  // 检测绿色物体位姿
	convertToMM(x, y); // 计算物体的中心位置
}

void VisionManager::get2DLocation_Red(cv::Mat img, float &x, float &y)
{
	this->curr_img = img;
	img_centre_x_ = img.rows / 2;
	img_centre_y_ = img.cols / 2;

	cv::Rect tablePos;

	detectTable(tablePos);  // 检测桌面轮廓

	detect2DObject_Red(x, y, tablePos); // 检测红色物体位姿
	convertToMM(x, y); // 计算物体的中心位置
}

void VisionManager::detectTable(cv::Rect &tablePos)
{
	// Extract Table from the image and assign values to pixel_per_mm fields
	//从图像中提取表并将值赋给pixel_per_mm字
	cv::Mat BGR[3];
	cv::Mat image = curr_img.clone();
	split(image, BGR);

	// 提取RGB图像中红色、绿色和蓝色通道
	cv::Mat gray_image_red = BGR[2];
	cv::Mat gray_image_green = BGR[1];
	cv::Mat gray_image_blue = BGR[0];

	// medianBlur 函数进行中值滤波处理
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_red, denoiseImage, 3);

	// Threshold the Image
	/* 下面的for循环是为了在红色和绿色两个通道中确认黑色桌子的位置，
									然后将桌子位置的点像素值设置为255，其他地方位置的像素值设置为0*/
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++)
		{
			// 获取红色、绿色、蓝色通道的阈值
			int editValue = binaryImage.at<uchar>(i, j);
			int editValue2 = gray_image_green.at<uchar>(i, j);
			int editValue3 = gray_image_blue.at<uchar>(i, j);
			
			//如果像素点在红色、绿色和蓝色通道中的阈值均在0-20之间，那么就判断为黑色
			if ((editValue >= 0) && (editValue < 20) && (editValue2 >= 0) && (editValue2 < 20) && (editValue3 >= 0) && (editValue3 < 20))
			{ // check whether value is within range.
				binaryImage.at<uchar>(i, j) = 255;
			}
			else
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());

	// Get the centroid of the of the blob
	// 先找像素值非零的位置
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);

	/* 再通过boundingRect函数计算轮廓的垂直边界最小矩形，矩形是与图像上下边界平行的
							                                                              通俗点说就是找到一个最小的矩形，把找到的形状包起来*/
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);

	// 计算桌面中心点位置
	// boundingRect返回四个值，x，y是矩阵左上点的坐标，wideth，height是矩阵的宽和高
	cv::Point pt;
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;

	//在中心点画一个红色(BGR)的圈圈标示
	cv::circle(image, pt, 2, cv::Scalar(0, 0, 255), 2, 8);

	// Update pixels_per_mm fields
	// 计算桌子的实际尺寸与像素尺寸的对应比例关系
	//table_length是桌子实际的长，bbox.height是像素长度
	pixels_permm_y = bbox.height / table_length;
	//table_length是桌子实际的宽，bbox.height是像素宽度
	pixels_permm_x = bbox.width  / table_breadth;

    tablePos = bbox;

	// Test the conversion values
	std::cout << "Pixels in y" << pixels_permm_y << std::endl;
	std::cout << "Pixels in x" << pixels_permm_x << std::endl;

	// Draw Contours - For Debugging
	// 画出桌面蓝色边框
	// 通过cv::findContours函数检测轮廓
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	// CV_RETR_TREE， 检测所有轮廓，所有轮廓建立一个等级树结构
	/*  CV_CHAIN_APPROX_SIMPLE 仅保存轮廓的拐点信息，把所有轮廓拐点处的点保存入contours
																															向量内，拐点与拐点之间直线段上的信息点不予保留*/
	// cv::Point(0, 0) 不设置偏移
	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(0, 255, 255); // 取色为黄色 (BGR)
		// 绘制边框
		cv::drawContours(image, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
	}

	cv::namedWindow("Table Detection", 0);
	cv::resizeWindow("Table Detection", 480, 480);
	cv::imshow("Table Detection", image);
	cv::waitKey(100);
}

void VisionManager::detect2DObject_Green(float &pixel_x, float &pixel_y, cv::Rect &tablePos)
{
	// Implement Color Thresholding and contour findings to get the location of object to be grasped in 2D
	cv::Mat image, gray_image_green;
	cv::Mat BGR[3];
	image = curr_img.clone();
	cv::split(image, BGR);

	gray_image_green = BGR[1];

	// Denoise the Image
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_green, denoiseImage, 3);

	// Threshold the Image
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++)
		{
			if((j<tablePos.x+3) || j>(tablePos.x+tablePos.width-3) || (i<tablePos.y+3) || i>(tablePos.y + tablePos.height-3))
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
			else
			{
				int editValue = binaryImage.at<uchar>(i, j);

				if ((editValue > 100) && (editValue <= 255))
				{ // check whether value is within range.
					binaryImage.at<uchar>(i, j) = 255;
				}
				else
				{
					binaryImage.at<uchar>(i, j) = 0;
				}
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());

	// Get the centroid of the of the blob
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pixel_x = bbox.x + bbox.width / 2;
	pixel_y = bbox.y + bbox.height / 2;

	// Test the conversion values
	std::cout << "pixel_x" << pixel_x << std::endl;
	std::cout << "pixel_y" << pixel_y << std::endl;

	// For Drawing
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 2, cv::Scalar(0, 255, 255), 4, 8);

	// Draw Contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(0, 255, 255);
		cv::drawContours(image, contours, i, color, 4, 8, hierarchy, 0, cv::Point());
	}

	cv::namedWindow("Green Centre point", 0);
	cv::resizeWindow("Green Centre point", 480, 480);
	cv::imshow("Green Centre point", image);
	cv::waitKey(100);
}

void VisionManager::detect2DObject_Red(float &pixel_x, float &pixel_y, cv::Rect &tablePos)
{
	// Implement Color Thresholding and contour findings to get the location of object to be grasped in 2D
	cv::Mat image, gray_image_red;
	cv::Mat BGR[3];
	image = curr_img.clone();
	cv::split(image, BGR);

	gray_image_red = BGR[2];

	// Denoise the Image
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_red, denoiseImage, 3);

	// Threshold the Image
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++)
		{
			if((j<tablePos.x+3) || j>(tablePos.x+tablePos.width-3) || (i<tablePos.y+3) || i>(tablePos.y + tablePos.height-3))
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
			else
			{
				int editValue = binaryImage.at<uchar>(i, j);

				if ((editValue > 100) && (editValue <= 255))
				{ // check whether value is within range.
					binaryImage.at<uchar>(i, j) = 255;
				}
				else
				{
					binaryImage.at<uchar>(i, j) = 0;
				}
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());

	// Get the centroid of the of the blob
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pixel_x = bbox.x + bbox.width / 2;
	pixel_y = bbox.y + bbox.height / 2;

	// Test the conversion values
	std::cout << "pixel_x" << pixel_x << std::endl;
	std::cout << "pixel_y" << pixel_y << std::endl;

	// For Drawing
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 2, cv::Scalar(0, 255, 255), 4, 8);

	// Draw Contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(0, 255, 255);
		cv::drawContours(image, contours, i, color, 4, 8, hierarchy, 0, cv::Point());
	}

	cv::namedWindow("Red Centre point", 0);
	cv::resizeWindow("Red Centre point", 480, 480);
	cv::imshow("Red Centre point", image);
	cv::waitKey(100);
}

void VisionManager::detect2DObject_Blue(float &pixel_x, float &pixel_y, cv::Rect &tablePos)
{
	// Implement Color Thresholding and contour findings to get the location of object to be grasped in 2D
	cv::Mat image, gray_image_blue;
	cv::Mat BGR[3];
	image = curr_img.clone();
	cv::split(image, BGR);

	gray_image_blue = BGR[0];

	// Denoise the Image
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_blue, denoiseImage, 3);

	// Threshold the Image
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++)
		{
			if((j<tablePos.x+3) || j>(tablePos.x+tablePos.width-3) || (i<tablePos.y+3) || i>(tablePos.y + tablePos.height-3))
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
			else
			{
				int editValue3 = gray_image_blue.at<uchar>(i, j);

				if ((editValue3 > 100) && (editValue3 <= 255))
				{ // check whether value is within range.
					binaryImage.at<uchar>(i, j) = 255;
				}
				else
				{
					binaryImage.at<uchar>(i, j) = 0;
				}
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());

	// Get the centroid of the of the blob
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pixel_x = bbox.x + bbox.width / 2;
	pixel_y = bbox.y + bbox.height / 2;

	// Test the conversion values
	std::cout << "pixel_x" << pixel_x << std::endl;
	std::cout << "pixel_y" << pixel_y << std::endl;

	// For Drawing
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 2, cv::Scalar(0, 255, 255), 4, 8);

	// Draw Contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(0, 255, 255);
		cv::drawContours(image, contours, i, color, 4, 8, hierarchy, 0, cv::Point());
	}

	cv::namedWindow("Blue Center point", 0);
	cv::resizeWindow("Blue Center point", 480, 480);
	cv::imshow("Blue Center point", image);
	cv::waitKey(100);
}

void VisionManager::convertToMM(float &x, float &y)
{
	// Convert from pixel to world co-ordinates in the camera frame
	// 计算目标中心位置
	// 将物体的中心点位置进行变换，计算出中心位置相对于摄像头采集到图像的中心位置的左标(以中心点为(0,0)坐标原点)
	x = (x - img_centre_x_) / pixels_permm_x;
	y = (y - img_centre_y_) / pixels_permm_y;
}

// Temporary Main Function for testing- This should go away later
// int main(int argc, char** argv ) {
// 	if ( argc != 2 )
//     {
//         printf("usage: VisionManager <Image_Path>\n");
//         return -1;
//     }

//     cv::Mat image;
//     image = cv::imread( argv[1], 1 );

//     if ( !image.data )
//     {
//         printf("No image data \n");
//         return -1;
//     }

//     float length = 0.3;
//     float breadth = 0.3;
//     float obj_x, obj_y;

//     VisionManager vm(length, breadth);
//     vm.get2DLocation(image, obj_x, obj_y);
//     std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
//     std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

//     cv::waitKey(0);
// }
