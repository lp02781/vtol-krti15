#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/Int16.h"
#include <iostream>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fstream>
#include <arpa/inet.h>
#include <vector>
#include <unistd.h>
#include <string>

using namespace std;
using namespace cv;

void *videoCapture ( void *ptr );   
void *videoStream ( void *ptr );   
Mat imgOriginal, imgTransmit;
VideoCapture cap(0);
int exit_loop = 0;
string gcs_ip;
int main (int argc, char** argv)
{
	double dM01,dM10,dArea;
	float posX,posY;
	geometry_msgs::Point cv;
	std_msgs::Int16 fm;
	int control_hsv = 0;

	// Initialize ROS
	ros::init (argc, argv, "image_processing");
	ros::NodeHandle n("~");  
	ros::Publisher pub_cv = n.advertise<geometry_msgs::Point>("/krti15/cv_point", 1000);
	ros::Publisher pub_flightmode = n.advertise<std_msgs::Int16>("/krti15/flight_mode", 1000);
	n.getParam("gcsip", gcs_ip);
	n.getParam("chsv", control_hsv);
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 240);
	
    if ( !cap.isOpened() ){  // if not success, exit program
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
    cap.read(imgOriginal);
    
	pthread_t video_capture;		// Thread untuk mengambil gambar
	pthread_create (&video_capture, NULL , videoCapture, NULL);	// Thread untuk membuka kamera
	
	pthread_t video_stream;		// Thread untuk mengambil gambar
	pthread_create (&video_stream, NULL , videoStream, NULL);	// Thread untuk membuka kamera

    int low = 225;
    int high = 255;
    
    int noise = 0;
    int holes = 20;
	
	if(control_hsv == 1){

	
	namedWindow("Object", CV_WINDOW_AUTOSIZE);
    //Create trackbars in "Object" window
    createTrackbar("Low", "Object", &low, 255);//Value (0 - 255)
    createTrackbar("High", "Object", &high, 255);

    createTrackbar("removes small noise", "Object", &noise, 20);
    createTrackbar("removes small holes", "Object", &holes, 20);
	
		
	}
	
	ROS_INFO("Starting Image Processing.");
	while(n.ok()){
       
		Mat imgGRAY;
        cvtColor(imgOriginal, imgGRAY, COLOR_BGR2GRAY);

        Mat imgThresholded;
        inRange(imgGRAY, low, high, imgThresholded);

        if(noise > 0) {
            //morphological opening (removes small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(noise, noise)) );
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(noise, noise)) );
        }

        if(holes > 0) {
            //morphological closing (removes small holes from the foreground)
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(holes, holes)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(holes, holes)) );
        }
        
        Mat canny_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		int largest_area=0;
 		int largest_contour_index=0;
 		Rect bounding_rect;

		/// Detect edges using canny
		Canny( imgThresholded, canny_output, 100, 100*2, 3 );
		/// Find contours
		findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		/// Draw contours
		Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ )
		{
			double a=contourArea( contours[i],false);  //  Find the area of contour
			if(a>largest_area){
				largest_area=a;
				largest_contour_index=i;                
				bounding_rect=boundingRect(contours[i]); 
			}
		}

		int posX, posY;
		posX = bounding_rect.x + (bounding_rect.width/2);
		posY = bounding_rect.y + (bounding_rect.height/2);
		
		if (posX != 0 && posY != 0){
			cv.x = posX;
			cv.y = posY;
			pub_cv.publish(cv);
		
			// if detect light, set flight mode to GUIDED (1)
			fm.data = 1;
			pub_flightmode.publish(fm);
		}
		
		else {
			// if doesn't detect light, set flight mode to AUTO (0)
			fm.data = 0;
			pub_flightmode.publish(fm);
		}
		
		imgTransmit = imgThresholded;
        if (waitKey(30) == 27){ //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            cout << "esc key is pressed by user" << endl;
            break; 
		}
        
       ros::spinOnce();
	}
	exit_loop = 1;
	pthread_join(video_capture, NULL);
	pthread_join(video_stream, NULL);
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


// Video UDP Streaming a bit buggy
void *videoStream ( void *ptr ){
	
	char encoded_image[100000];
	vector<uchar> buff;
	vector<int> param = vector<int>(2);
	int clientSocket, portNum, bytes, x, server_port;
	unsigned int nBytes;
	struct sockaddr_in serverAddr;
	struct sockaddr_storage serverStorage;
	socklen_t addr_size;
	const char *server_ip = gcs_ip.c_str();
	
	param[0] = CV_IMWRITE_JPEG_QUALITY;			// set tipe encoding 
	param[1] = 35;					// set kualitas encoding
	
	/* Inisialisasi Socket Client */				
	server_port = 7891;				// Port Server yang dituju
	clientSocket = socket(PF_INET, SOCK_DGRAM, 0);	// membuka koneksi UDP
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(server_port);		// Membuka Port Server
	serverAddr.sin_addr.s_addr = inet_addr(server_ip);	// Membuka IP Server
	memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);  
	addr_size = sizeof serverAddr;	
	/* Inisialisasi Socket Client */
	
	usleep(2000000);
	while(exit_loop == 0){
			
		imencode(".jpeg",imgTransmit, buff,param);	// meng-encode gambar
			  
		for (x = 0; x < buff.size(); x++){				// merubah data gambar dari tipe vector ke array
			encoded_image[x] = buff[x];
		}
		
		nBytes = buff.size();
		if (nBytes < 65000){
			bytes = sendto(clientSocket,encoded_image,nBytes,0,(struct sockaddr *)&serverAddr,addr_size);	// mengirim data ke server
		}
		memset(&encoded_image[0], 0, nBytes);
		usleep(100000);
	}
	
	pthread_exit(0); 
}
