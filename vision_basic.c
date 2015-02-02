	#include "opencv2/opencv.hpp"
	#include <raspicam/raspicam_cv.h>
	#include <iostream>

	using namespace std;
	using namespace cv;
	int main ()
	{
		cv::Mat image;
		namedWindow("window",1);
	    raspicam::RaspiCam_Cv Camera;
	    if (!Camera.open()) 
	    {
	    	cerr<<"Error opening the camera"<<endl;
	    	return -1;
	    }		
	     
	    //set camera params
	    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
	    camera.set(CV_CAP_PROP_FRAME_HEIGHT,320);
		camera.set(CV_CAP_PROP_FRAME_WIDTH,240);

	for ( int i=0; i<nCount; i++ ) {
	        Camera.grab();
	        Camera.retrieve ( image);
	        imshow( "window", image );
	        if(waitKey(10)==(int)'q')break;
		}

		Camera.release();
		destroyAllWindows();
		cout << "Saindo" << endl;
		return 0;
	}

