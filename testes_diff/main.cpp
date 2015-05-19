#include <iostream>
#include <sstream>
#include <math.h>
#include <string.h>
#include <opencv.hpp>
#define ERODE

int FAIXA_H =5;
int FAIXA_S =25;
int FAIXA_V =42;

using namespace std;
using namespace cv;

Mat image_HSV;
int _x,_y;
int v1=20,v2=90,N=11;
Vec3b color;
Point center_screen;
Point2f center;                                     ///Variavel que recebe o centro do blob.

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    //http://opencv-srf.blogspot.com.br/2011/11/mouse-events.html
    static bool flag_Release = false;
    if  ( event == EVENT_LBUTTONDOWN )
    {
        flag_Release = true;
        _x=x;
        _y=y;
        //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_LBUTTONUP )
    {
        flag_Release = false;
        color = image_HSV.at<Vec3b>(Point(_x,_y));
        //Mat M(200,200,CV_8UC3,Scalar((int)color[0],255,100));
        //cvtColor(M,M,CV_HSV2BGR);
        //imshow("Color",M);
        cout << "color_HSV: [" << (int)color[0]<< ","<< (int)color[1]<< ","<< (int)color[2]<<"]" << endl;
    }
    else if ( event == EVENT_MOUSEMOVE && flag_Release)
    {
        _x=x;
        _y=y;
    }
}
void show_xy_color(Mat img,int x,int y)
{
    Vec3b color = img.at<Vec3b>(Point(x,y));
    Mat M(200,200,CV_8UC3,Scalar((int)color[0],255,100));
    cvtColor(M,M,CV_HSV2BGR);
    //imshow("Color",M);
    cout << "color_HSV: [" << (int)color[0]<< ","<< (int)color[1]<< ","<< (int)color[2]<<"]" << endl;
}

int main()
{
    VideoCapture cap;
    for(int i=0; i<2; i++)
    {
        cap.open(i);
        if (cap.isOpened()) break;
    }

    if (!cap.isOpened())
    {
        cerr<<"Error opening the camera"<<endl;
        return -1;
    }
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,320);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,240);
    center_screen = Point(cap.get(CV_CAP_PROP_FRAME_WIDTH)/2,cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);
        color[0] = 103;         ///Azul
    //    cap.set(CV_CAP_PROP_GAIN, 48);
//    cap.set(CV_CAP_PROP_BRIGHTNESS, 10);

    namedWindow("window",1);
    createTrackbar("H","window", &FAIXA_H, 90);
    createTrackbar("S","window", &FAIXA_S, 126);
    createTrackbar("V","window", &FAIXA_V, 126);
    setMouseCallback("window", CallBackFunc, NULL);


    Mat frame,aux_hsv,result,aux;

    vector<Mat> HSV_chanells;

    vector<vector<Point> > contours;
    vector<Point> approx;

    for(;;)
    {
        cap >> frame;
//        GaussianBlur(frame, frame, Size(5, 5), 2, 1 );//aplique um filtro
        medianBlur(frame,frame,5);
        cvtColor(frame,aux_hsv,CV_BGR2HSV);
        aux_hsv.copyTo(image_HSV);

        /********************************************************************************************************************************/
        #ifdef ERODE
        ///Erosão
        int erosion_size = 3;
        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                             Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                             Point( erosion_size, erosion_size ) );
        /// Apply the erosion operation
        //erode( result, result, element );
//        erode( frame, frame, element );
        dilate( frame, frame, element );
        #endif
        /********************************************************************************************************************************/
//        threshold(HSV_chanells[0],HSV_chanells[0],min(179,(int)color[0]+FAIXA_H),255,THRESH_TOZERO_INV);
//        threshold(HSV_chanells[0],HSV_chanells[0],max(0,(int)color[0]-FAIXA_H),255,THRESH_TOZERO);
//        threshold(HSV_chanells[0],result,1,255,THRESH_BINARY);
        inRange(aux_hsv,Scalar(color[0]-FAIXA_H,color[1]-FAIXA_S,color[2]-FAIXA_V),Scalar(color[0]+FAIXA_H,color[1]+FAIXA_S,color[2]+FAIXA_V),result);
        result.copyTo(aux);

        //imshow("debug",HSV_chanells[2]);
        //MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE;
        //show_xy_color(image_HSV,_x,_y);
        //Mat img_canny(frame.size(), CV_8U);//nova matriz
        //Canny(result, img_canny, v1, v2);
        findContours(result,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
//        if(contours.size()>0)
//            cout<<contours.size()<<" blob encontrado"<<endl;

        //            count_frames_without_blob=0;
        /************************************************************
         * Segmentação dos blobs
         ***********************************************************/
        int blob_index=0;
        int blob_area=0;
        int area_min = 200;
        /***********************************************************
         * Separa o maior blob para segmentação
        ***********************************************************/
        for( size_t i = 0; i < contours.size() ; i++ )
        {
            if(fabs(contourArea(Mat(contours[i]))) >= area_min /*&& fabs(contourArea(Mat(contours[i]))) < 10000*/)
            {
                blob_area = fabs(contourArea(Mat(contours[i])));
                blob_index = i;
            }
        }

        if(blob_area >= area_min)
        {
            //cout << "Área: "<< blob_area <<endl;
            approxPolyDP(contours[blob_index], approx, arcLength(Mat(contours[blob_index]), true)*0.02, true);
        /************************************************************
         * Extração do centro do blob
         ************************************************************/
        //Point2f center;                                     ///Variavel que recebe o centro do blob.
        float radius;                                           ///Variavel auxiliar.
        minEnclosingCircle(approx,center,radius);           ///Acha o centro do blob.
        //circle( result, center, (int)radius, 255/*cv::Scalar(255,200,100)*/, 1);
        circle( frame, center, (int)radius, cv::Scalar(255,0,0), 2);      ///Circulo que marca o blob.

        cout <<"centro:("<< (int)center.x<<", "<<(int)center.y<<") / Area="<<blob_area<<endl;
        }

        circle(frame,center_screen,5,Scalar(10,255,50),3,2);        ///Circulo qua marca o centro.
        circle(frame,Point(_x,_y),1,Scalar(100,255,50),3,2);        ///Circulo que marca o ponto clicado.

        if(center.x >center_screen.x)line(frame,center_screen,center,Scalar(0,255,0),2);
        else line(frame,center_screen,center,Scalar(0,0,255),2);

        imshow("result",aux);
        imshow( "window", frame );

        int key = waitKey(10);
        if(key==(int)'q')break;     ///Sair com letra q

    }
    destroyAllWindows();
    return 0;
}
/******************************************************************************/

/******************************************************************************/

