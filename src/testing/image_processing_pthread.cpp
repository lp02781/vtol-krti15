#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/Int16.h"
#include <iostream>
#include <pthread.h>

using namespace std;
using namespace cv;

void *videoCapture ( void *ptr );   
Mat imgOriginal;
VideoCapture cap(1);	//Open the webcam
int exit_loop = 0;

int main (int argc, char** argv)
{
	double dM01,dM10,dArea;
	float posX,posY;
	geometry_msgs::Point cv;
	std_msgs::Int16 fm;
	 
	// Initialize ROS
	ros::init (argc, argv, "img_proc");
	ros::NodeHandle n;  
	ros::Publisher pub_cv = n.advertise<geometry_msgs::Point>("krti15/cv_point", 1000);
	ros::Publisher pub_flightmode = n.advertise<std_msgs::Int16>("krti15/flight_mode", 1000);

    if ( !cap.isOpened() ){  // if not success, exit program
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
    cap.read(imgOriginal);
    
	pthread_t video_capture;		// Thread untuk mengambil gambar
	pthread_create (&video_capture, NULL , videoCapture, NULL);	// Thread untuk membuka kamera

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 0;
	int iHighH = 10;

	int iLowS = 034; 
	int iHighS = 255;

	int iLowV = 137;
	int iHighV = 255;

	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);

	//Create a black image with the size as the camera output
	Mat imgCircle = Mat::zeros( imgOriginal.size(), CV_8UC3 );
	Mat imgDetection;
	while(n.ok()){
       
		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		dM01 = oMoments.m01;
		dM10 = oMoments.m10;
		dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000){
			//calculate the position of the ball
			posX = dM10 / dArea;
			posY = dM01 / dArea;        
			if (posX >= 0 && posY >= 0){
				
			//Draw a red circle in the current point
			circle( imgCircle, Point(posX, posY), 32.0, Scalar( 0, 0, 255 ), 7, 0 );
			cv.x = posX;
			cv.y = posY;
			pub_cv.publish(cv);
			
			// if detect light, set flight mode to GUIDED (1)
			fm.data = 1;
			pub_flightmode.publish(fm);
			
			ROS_INFO_STREAM( 
			"\nX Value = " << cv.x
			<< "\nY Value = " << cv.y
			<< "\nFM Value = " << fm.data ) ;
			}
			
		}
		
		else {
			
			// if doesn't detect light, set flight mode to AUTO (0)
			fm.data = 0;
			pub_flightmode.publish(fm);
			ROS_INFO_STREAM("\nFM Value = " << fm.data ) ;
		
		}

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imgDetection = imgOriginal + imgCircle;
		imgCircle = Mat::zeros( imgOriginal.size(), CV_8UC3 ); 
		imshow("Original", imgDetection); //show the original image
		
        if (waitKey(30) == 27){ //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            cout << "esc key is pressed by user" << endl;
            break; 
		}
        
       ros::spinOnce();
	}
	exit_loop = 1;
	pthread_join(video_capture, NULL);
}

void *videoCapture ( void *ptr ){

	while(exit_loop == 0){
		
		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
			cout << "Cannot read a frame from video stream" << endl;
			break;
        }
	}
	pthread_exit(0); 
}
