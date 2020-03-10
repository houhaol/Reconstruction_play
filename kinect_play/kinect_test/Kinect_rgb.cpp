// kinect_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "pch.h"
#include <Kinect.h>      
#include <iostream>
#include <opencv2\highgui.hpp>  

using namespace std;
using namespace cv;
int main(void)
{
	IKinectSensor* mySensor = nullptr; // Get kinect Sensor
	GetDefaultKinectSensor(&mySensor);

	mySensor->Open(); // Open kinect sensor

	// Get depth data from kinect sensor
	IColorFrameSource* mySource = nullptr;
	mySensor->get_ColorFrameSource(&mySource);

	// Get depth image height and width
	int height = 0, width = 0;
	IFrameDescription* myDescription = nullptr;
	mySource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);
	myDescription->Release();
	myDescription = nullptr;

	// Establish image matrix
	Mat mColorImg(height, width, CV_8UC4);
	namedWindow("ColorImage");

	// Open depth data reader
	IColorFrameReader* myReader = nullptr;
	mySource->OpenReader(&myReader);

	// Loop
	while (1)
	{
		// Get the lattest image
		IColorFrame* myFrame = nullptr;
		if (myReader->AcquireLatestFrame(&myFrame) == S_OK)
		{
			//// Save image into image matrix and convert to BGRA format
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
