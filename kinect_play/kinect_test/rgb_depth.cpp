

 //kinect_test.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "pch.h"
#include <Kinect.h>      
#include <iostream>
#include <opencv2\highgui.hpp>  
 
#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>   
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <opencv2/opencv.hpp>  
#include <pcl/point_types.h>  
#include <pcl/point_cloud.h>  

using namespace std;
using namespace cv;


IKinectSensor* mySensor = nullptr; // Get kinect Sensor
ICoordinateMapper* myMapper = nullptr;
const int iWidth = 512, iHeight = 424;
CameraSpacePoint depth2xyz[iWidth*iHeight];
ColorSpacePoint depth2rgb[iWidth*iHeight];

void viewerOneoff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1, 1, 1);
}

bool initKinect()
{
	if (FAILED(GetDefaultKinectSensor(&mySensor))) return false;
	if (mySensor)
	{
		mySensor->get_CoordinateMapper(&myMapper);
		mySensor->Open(); // Open kinect sensor
		return true
	}
	else
		return false;
}

void getPointCloudFromImage(Mat DepthImage, Mat RgbImage, pcl::PointCloud<pcl::PointXYZRGB> * cloud_out)
{
	myMapper->MapDepthFrameToCameraSpace(iWidth*iHeight, reinterpret_cast<UINT16*>(DepthImage.data), iWidth*iHeight, depth2xyz);
	myMapper->MapDepthFrameToColorSpace(iWidth*iHeight, reinterpret_cast<UINT16*>(DepthImage.data), iWidth*iHeight, depth2rgb);

	cloud_out.height = 1;
	cloud_out.is_dense = 1;

	for (size_t i = 0; i < iWidth; i++)
	{
		for (size_t j = 0; j < iHeight; j++)
		{
			pcl::PointXYZRGB pointtemp;
			if (depth2xyz[i + j * iWidth].Z > 0.5)//?
			{
				pointTemp.x = depth2xyz[i + j * iWidth].X;
				pointTemp.y = depth2xyz[i + j * iWidth].Y;
				pointTemp.z = depth2xyz[i + j * iWidth].Z;
				int X = static_cast<int>(depth2rgb[j * iWidth + i].X);
				int Y = static_cast<int>(depth2rgb[j * iWidth + i].Y);
				if (X > 0 && Y > 0 && X < 1920 && X < 1080)
				{
					Vec3b* PixelsRGBImage = RgbImage.ptr<Vec3b>(Y);
					pointTemp.g = PixelsRGBImage[X][0];
					pointTemp.b = PixelsRGBImage[X][1];
					pointTemp.r = PixelsRGBImage[X][2];
					cloud_out.push_back(pointTemp);
				}
				else continue;
			}
		}
	}

}

 Get depth data from kinect sensor
IColorFrameSource* mySource = nullptr;
mySensor->get_ColorFrameSource(&mySource);

 Get depth image height and width
int height = 0, width = 0;
IFrameDescription* myDescription = nullptr;
mySource->get_FrameDescription(&myDescription);
myDescription->get_Height(&height);
myDescription->get_Width(&width);
myDescription->Release();
myDescription = nullptr;

 Establish image matrix
Mat mColorImg(height, width, CV_8UC4);
namedWindow("ColorImage");

 Open depth data reader
IColorFrameReader* myReader = nullptr;
mySource->OpenReader(&myReader);

int main(void)
{
	initKinect();
	Mat RgbImage = imread("pass");
	Mat DepthImage = imread("pass");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	getPointCloudFromImage(DepthImage, RgbImage, *cloud_out);
	pcl::visualization::CloudViewer viewerG("Cloud view");
	pcl::PLYWriter writer;
	writer.write("pass.ply", *cloud_out);
	viewerG.runOnVisualizationThreadOnce(viewerOneOff);
	viewerG.showCloud(cloud_out);
	while (true) if (cv::waitKey(30) == VK_ESCAPE) break;
	return 0;
	 Loop
	while (1)
	{
		 Get the lattest image
		IColorFrame* myFrame = nullptr;
		if (myReader->AcquireLatestFrame(&myFrame) == S_OK)
		{
			// Save image into image matrix and convert to BGRA format
			myFrame->CopyConvertedFrameDataToArray(height * width * 4, (BYTE*)mColorImg.data, ColorImageFormat_Bgra);
			imshow("ColorImage", mColorImg);
			myFrame->Release();
		}
		if (waitKey(30) == VK_ESCAPE) {
			break;
		}
	}
	myReader->Release();
	myReader = nullptr;
	mySource->Release();
	mySource = nullptr;
	mySensor->Close(); //Close sensor
	mySensor->Release();
	mySensor = nullptr;

	return 0;
}
