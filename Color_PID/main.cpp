#include <iostream>
#include <sstream>
#include <math.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#define _RASPI_
//#define ERODE
#define BLOB_AREA_MIN 2000
#define BLOB_AREA_MAX 4000

#ifdef _RASPI_
#include <wiringSerial.h>
#endif // _RASPI_

using namespace std;
using namespace cv;

/******************************************/
const int baud_rate = 9600;
int device;
char dev1[]="/dev/ttyUSB0",dev2[]="/dev/ttyAMA0";
/******************************************/

/******************************************/
float p_x;
float p_y;
float KP = 0.1;
float KI = 0.1;
/******************************************/

/******************************************/
int FAIXA_H =5;
int FAIXA_S =25;
int FAIXA_V =42;
/******************************************/

/******************************************/
Mat image_HSV;
int _x,_y;
int v1=20,v2=90,N=11;
Vec3b color;
Point center_screen;
Point2f center;                                     ///Variavel que recebe o centro do blob.
int locked_target = 0;
/******************************************/

/******************************************************************************/

class pid
{
    public:
	float kp;
	float ki;
	float kd;
	float error;
	float setpoint;
	float input;
	float output;
	float Iterm;
	float outMin;
	float outMax;
	float init;
	void set_limits(float a,float b);
	float compute(float in);
};

void pid::set_limits(float a,float b)
{
	if(a<b)
	{
		outMin = a;
		outMax = b;
	}
	else
	{
		outMin = b;
		outMax = a;
	}
}

float pid::compute(float in)
{
    input = in;
    error = setpoint - input;
    Iterm += (ki*error);
    if(Iterm> outMax) Iterm= outMax;
    else if(Iterm< outMin) Iterm= outMin;
    output = kp * error + Iterm + init;
    if(output > outMax) output = outMax;
    else if(output < outMin) output = outMin;
    return output;
}
/******************************************************************************/
int _map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

union int2_tochar
{
    short int _int;
    char _char[2];
} info_x,info_y;

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
/******************************************************************************/
int main()
{

#ifdef _RASPI_
    device = serialOpen(dev2, baud_rate);
#endif // _RASPI_

    pid pid_x,pid_y;
    pid_x.kp = 0.09;
    pid_x.ki = 0.03;
    pid_x.set_limits(0.0,180.0);
    pid_x.setpoint = (float)cap.get(CV_CAP_PROP_FRAME_WIDTH)/2;
    pid_x.init = 90;
    pid_y.kp = 0.09;
    pid_y.ki = 0.03;
    pid_y.set_limits(0.0,180.0);
    pid_y.setpoint = (float)cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2;
    pid_y.init = 110;

    VideoCapture cap;
    cap.open();
    if (!cap.isOpened())
    {
        cerr<<"Error opening the camera"<<endl;
        return -1;
    }
    
    #ifdef _RASPI_
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,320);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,240);
    #endif // RASPI


    center_screen = Point(cap.get(CV_CAP_PROP_FRAME_WIDTH)/2,
    cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);

    color[0] = 103;         ///Azul

//	  cap.set(CV_CAP_PROP_GAIN, 48);
//    cap.set(CV_CAP_PROP_BRIGHTNESS, 10);

    Mat frame,aux_hsv,result;
    vector<Mat> HSV_chanells;
    vector<vector<Point> > contours;
    vector<Point> approx;
    bool obj_detected = false;

    namedWindow("window",1);
    setMouseCallback("window", CallBackFunc, NULL);
    for(;;)
    {
        cap >> frame;
//      GaussianBlur(frame, frame, Size(5, 5), 2, 1 );//aplique um filtro
        medianBlur(frame,frame,5);
        cvtColor(frame,aux_hsv,CV_BGR2HSV);
        aux_hsv.copyTo(image_HSV);

        /*****************************************************
		*Erosão(opcional)
		******************************************************/
        #ifdef ERODE
        ///Erosão
        int erosion_size = 3;
        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                             Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                             Point( erosion_size, erosion_size ) );
        /// Apply the erosion operation
        //erode( result, result, element );
        erode( frame, frame, element );
        //dilate( frame, frame, element );
        #endif
        
		/*****************************************************
		*Busca por cor
		******************************************************/
        inRange(aux_hsv,Scalar(color[0]-FAIXA_H,color[1]-FAIXA_S,
			color[2]-FAIXA_V),Scalar(color[0]+FAIXA_H,
			color[1]+FAIXA_S,color[2]+FAIXA_V),result);

        findContours(result,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);

        /************************************************************
         * Segmentação dos blobs
         ***********************************************************/
        int blob_index=0;
        int blob_area=0;
        int area_min = 300;

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
            approxPolyDP(contours[blob_index], approx, 
				arcLength(Mat(contours[blob_index]), true)*0.02, true);

        /************************************************************
        * Extração do centro do blob
        ************************************************************/
        float radius;                                           ///Variavel auxiliar.
        minEnclosingCircle(approx,center,radius);           ///Acha o centro do blob.
//        circle( result, center, (int)radius, 255/*cv::Scalar(255,200,100)*/, 1);
//        circle( frame, center, (int)radius, cv::Scalar(255,0,0), 2);      ///Circulo que marca o blob.

		/*****************************************************
		* Calculo de correção PID para o carro
		******************************************************/
	locked_target +=2;
	if(locked_target>=20)locked_target=20
        float out_x = pid_x.compute(center.x);
        float out_y = pid_y.compute(center.y);
        cout<<blob_area<<endl;

        ///Sinaliza se esta perto ou longe do objeto
        int target_dist = 0;
        if(blob_area<BLOB_AREA_MIN)
        {
            circle(frame,center,2,Scalar(0,0,255),3);
            target_dist = 1;///Aproximar
        }
        else if(blob_area>BLOB_AREA_MIN && blob_area<BLOB_AREA_MAX)
        {
            circle(frame,center,2,Scalar(0,255,255),3);
            target_dist = 0;///Ficar onde esta
        }
        if(blob_area>BLOB_AREA_MAX)
        {
            circle(frame,center,2,Scalar(255,0,0),3);
            target_dist = 2;///Afastar
        }
        /****************************************/
        #ifdef _RASPI_
        serialPutchar(device,(unsigned char)200);
        serialPutchar(device,(unsigned char)(int)out_x);
        serialPutchar(device,(unsigned char)201);
        serialPutchar(device,(unsigned char)(int)out_y);
        serialPutchar(device,(unsigned char)202);
        serialPutchar(device,(unsigned char)target_dist);
        #endif // _RASPI_
        }
//        circle(frame,center_screen,5,Scalar(10,255,50),3,2);        ///Circulo qua marca o centro.
//        circle(frame,Point(_x,_y),1,Scalar(100,255,50),3,2);        ///Circulo que marca o ponto clicado.
//        if(center.x >center_screen.x)line(frame,center_screen,center,Scalar(0,255,0),2);
//        else line(frame,center_screen,center,Scalar(0,0,255),2);


#ifdef _RASPI_
        /*****************************************************
        * Sem detecção de blobs
        ******************************************************/
        locked_target -= 1;
        if(locked_target==0)
        {
           serialPutchar(device,(unsigned char)203);
        }
        if(locked_target<0)locked_target=0;
#endif // _RASPI_
		/*****************************************************
                * Exibição das imagens
		******************************************************/

#ifndef __RASPI_
        imshow("result",aux);
        #endif // _RASPI_
        imshow( "window", frame );

        int key = waitKey(10);
        if(key==(int)'q')break;     ///Sair com letra q

    }
#ifdef _RASPI_
    serialClose (device);
#endif
    destroyAllWindows();
    return 0;
}
/******************************************************************************/

/******************************************************************************/

