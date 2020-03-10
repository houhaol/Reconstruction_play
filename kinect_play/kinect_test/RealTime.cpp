// kinect_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "pch.h"
#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include <pcl\visualization\cloud_viewer.h> 
#include <pcl\visualization\pcl_visualizer.h>
#include <opencv2\highgui.hpp>
#include <pcl\point_cloud.h>
#include <pcl\io\ply_io.h>
#include <pcl\io\pcd_io.h>

using namespace cv;
using namespace std;


IKinectSensor* pSensor;
ICoordinateMapper *pMapper;
const int iDWidth = 512, iDHeight = 424; // Depth image size
const int iCWidth = 1920, iCHeight = 1080;// RGB image size
CameraSpacePoint depth2xyz[iDWidth*iDHeight];
ColorSpacePoint depth2rgb[iCWidth*iCHeight];


void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0, 0, 0);
}

// Load Kinect
bool initKinect()
{
	if (FAILED(GetDefaultKinectSensor(&pSensor))) return false;
	if (pSensor)
	{
		pSensor->get_CoordinateMapper(&pMapper);
		pSensor->Open();
		cout << "open camera" << endl;
		return true;
	}
	else return false;
}

// Depth data
Mat DepthData()
{
	IDepthFrameSource* pFrameSource = nullptr;
	pSensor->get_DepthFrameSource(&pFrameSource);
	IDepthFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);
	IDepthFrame* pFrame = nullptr;
	Mat mDepthImg(iDHeight, iDWidth, CV_16UC1);

	// Show depth image
	/*Mat mDepthImg(iDHeight, iDWidth, CV_16UC1);
	Mat mImg8bit(IDHeight, iDWidth, CV_8UC1);
	namedWindow("DepthImage");*/
	while (true)
	{
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			pFrame->CopyFrameDataToArray(iDWidth * iDHeight, reinterpret_cast<UINT16*>(mDepthImg.data));
			cout << "obtain depth" << endl;

			// Show depth image
			//myFrame->CopyFrameDataToArray(iDWidth * iDHeight, reinterpret_cast<UINT16*> (mDepthImg.data)); // convert data type
			//mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax); // input args: First, output matrix; 2. conversion type; 3. scale factor;
			//imshow("DepthImage", mImg8bit);

			pFrame->Release();
			return mDepthImg;
			break;
		}
	}
}

//Obatin RGB image
Mat RGBData()
{
	IColorFrameSource* pFrameSource = nullptr;
	pSensor->get_ColorFrameSource(&pFrameSource);
	IColorFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);
	IColorFrame* pFrame = nullptr;
	Mat mColorImg(iCHeight, iCWidth, CV_8UC4);
	while (true)
	{
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			pFrame->CopyConvertedFrameDataToArray(iCWidth * iCHeight * 4, mColorImg.data, ColorImageFormat_Bgra);
			cout << "obtain rgb" << endl;
			pFrame->Release();
			return mColorImg;
			break;
		}
	}
}


int main()
{
	initKinect();
	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	Mat mColorImg;
	Mat mDepthImg;
	//while (1)
	//{
	mColorImg = RGBData();
	mDepthImg = DepthData();
		
	pMapper->MapDepthFrameToColorSpace(iDHeight*iDWidth, reinterpret_cast<UINT16*>(mDepthImg.data), iDHeight*iDWidth, depth2rgb);// 
	pMapper->MapDepthFrameToCameraSpace(iDHeight*iDWidth, reinterpret_cast<UINT16*>(mDepthImg.data), iDHeight*iDWidth, depth2xyz);//深度图到相机三维空间的映射
	//for (int i = 0; i < iDWidth*iDHeight; i++)
	//{
	//	cout << i << ":  " << "X=" << depth2rgb[i].X << ";  Y=" << depth2rgb[i].Y<<endl;
	//}

	float maxX = depth2xyz[0].X, maxY = depth2xyz[0].Y, maxZ = depth2xyz[0].Z;
	float minX = depth2xyz[0].X, minY = depth2xyz[0].Y, minZ = depth2xyz[0].Z;
	for (size_t i = 0; i < iDWidth; i++)
	{
		for (size_t j = 0; j < iDHeight; j++)
		{
			pcl::PointXYZRGBA pointTemp;
			if (depth2xyz[i + j * iDWidth].Z > 0.5 && depth2rgb[i + j * iDWidth].X < 1920 && depth2rgb[i + j * iDWidth].X>0 && depth2rgb[i + j * iDWidth].Y < 1080 && depth2rgb[i + j * iDWidth].Y>0)
			{
				pointTemp.x = -depth2xyz[i + j * iDWidth].X;
				if (depth2xyz[i + j * iDWidth].X > maxX) maxX = -depth2xyz[i + j * iDWidth].X;
				if (depth2xyz[i + j * iDWidth].X < minX) minX = -depth2xyz[i + j * iDWidth].X;
				pointTemp.y = depth2xyz[i + j * iDWidth].Y;
				if (depth2xyz[i + j * iDWidth].Y > maxY) maxY = depth2xyz[i + j * iDWidth].Y;
				if (depth2xyz[i + j * iDWidth].Y < minY) minY = depth2xyz[i + j * iDWidth].Y;
				pointTemp.z = depth2xyz[i + j * iDWidth].Z;
				if (depth2xyz[i + j * iDWidth].Z != 0.0)
				{
					if (depth2xyz[i + j * iDWidth].Z > maxZ) maxZ = depth2xyz[i + j * iDWidth].Z;
					if (depth2xyz[i + j * iDWidth].Z < minZ) minZ = depth2xyz[i + j * iDWidth].Z;
				}
				pointTemp.b = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[0];
				pointTemp.g = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[1];
				pointTemp.r = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[2];
				pointTemp.a = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[3];
				cloud->push_back(pointTemp);
			}
		}
	}
	imshow("RGB", mColorImg);
	//imwrite("test.jpg", mColorImg);
	viewer.showCloud(cloud);
	mColorImg.release();
	mDepthImg.release();
	pcl::io::savePLYFileASCII("output.ply",*cloud);
	cloud->clear();
	/*if (cv::waitKey(30) == VK_ESCAPE) {
		break;
	}*/
	//}
	pSensor->Close(); //Close sensor
	pSensor->Release();
	pSensor = nullptr;
	return 0;
}
