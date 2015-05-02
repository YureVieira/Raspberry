#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;
int main ()
{
    cv::Mat image;
    namedWindow("window",1);
    VideoCapture cap;
    for(int i=0; i<10; i++)
    {
        cap.open(i);
        if (cap.isOpened()) break;
    }

    if (!cap.isOpened())
    {
        cerr<<"Error opening the camera!"<<endl;
        return -1;
    }
    //set camera param
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
    for (;;)
    {
        cap >> image;
        imshow( "window", image );
        if(waitKey(10)==(int)'q')break;
    }
    destroyAllWindows();
    cout << "Saindo" << endl;
    return 0;
}

//can not change format with camera already opened
