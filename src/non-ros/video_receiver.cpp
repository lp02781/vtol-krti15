#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
	
	int udpSocket, nBytes, server_port;
	char buffer[100000];
	struct sockaddr_in serverAddr, clientAddr;
	struct sockaddr_storage serverStorage;
	socklen_t addr_size, client_addr_size;
	Mat img_stream;
	
	
	/* Socket Server Initialization*/
	server_port = 7891;								// Set Port Number
	udpSocket = socket(AF_INET, SOCK_DGRAM, 0);		// Set to UDP
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_port = htons(server_port);		// Set Port Number
	memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);  
	bind(udpSocket, (struct sockaddr *) &serverAddr, sizeof(serverAddr));
	addr_size = sizeof serverStorage;
	/* Socket Server Initialization */
	
	namedWindow("Video Streamer", WINDOW_NORMAL);
	
  
	while(1){
    
		nBytes = recvfrom(udpSocket,buffer,100000,0,(struct sockaddr *)&serverStorage, &addr_size); // receiving Data from Quadcopter
		
		cout << nBytes << " Bytes Received" << endl;
		Mat img_buff(Size(640, 360), CV_8U, buffer);				// declare variable to store received image
		img_stream = imdecode(img_buff, CV_LOAD_IMAGE_COLOR);		// decode the received data
		if (!img_stream.empty()){
			imshow("Video Streamer", img_stream);					// display the image
		}
		
		if (waitKey(30) == 27){
			cout << "esc key is pressed by user" << endl;
			break; 
		}

	}

  return 0;
}
