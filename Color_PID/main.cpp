#include <iostream>
#include <sstream>
#include <math.h>
#include <string.h>
#include <opencv.hpp>
#define _RASPI_
//#define ERODE

#ifdef _RASPI_
#include <wiringSerial.h>
#endif // _RASPI_

int FAIXA_H =5;
int FAIXA_S =25;
int FAIXA_V =42;
float p_x;
float p_y;


using namespace std;
using namespace cv;

Mat image_HSV;
int _x,_y;
int v1=20,v2=90,N=11;
Vec3b color;
Point center_screen;
Point2f center;                                     ///Variavel que recebe o centro do blob.
short int error_x,error_y;

const int baud_rate = 9600;
int device;
char dev1[]="/dev/ttyUSB0",dev2[]="/dev/ttyAMA0";
/******************************************************************************/
int _map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/******************************************************************************/
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
	float v_init;

	void set_limits(float a,float b);
	float compute(float in);
};
/******************************************************************************/

/******************************************************************************/
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
/******************************************************************************/

/******************************************************************************/
float pid::compute(float in)
{
	input = in;
	error = setpoint - input;
	Iterm += (ki*error);
	if(Iterm> outMax) Iterm= outMax;
      else if(Iterm< outMin) Iterm= outMin;
      output = kp * error + Iterm+v_init;
    if(output > outMax) output = outMax;
    else if(output < outMin) output = outMin;
    return output;
}
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
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

int main()
{
#ifdef _RASPI_
    device = serialOpen(dev2, baud_rate);
#endif // _RASPI_
    VideoCapture cap;
    cap.open(0);

    if (!cap.isOpened())
    {
        cerr<<"Error opening the camera"<<endl;
        return -1;
    }
    #ifdef _RASPI_
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,320);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,240);
    #endif // RASPI

//    cap.set(CV_CAP_PROP_FRAME_HEIGHT,640);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH,480);
    center_screen = Point(cap.get(CV_CAP_PROP_FRAME_WIDTH)/2,cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);
    color[0] = 103;         ///Azul
    pid pid_x,pid_y;
    pid_x.kp = 0.07;
    pid_x.ki = 0.03;
    pid_x.set_limits(0.0,180.0);
    pid_x.setpoint = (float)cap.get(CV_CAP_PROP_FRAME_WIDTH)/2;
    pid_x.v_init = 90;
    pid_y.kp = 0.07;
    pid_y.ki = 0.03;
    pid_y.set_limits(0.0,180.0);
    pid_y.setpoint = (float)cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2;
    pid_y.v_init = 30;
    cout<<"X(Kp="<<pid_x.kp<<", Ki="<<pid_x.ki<<")/   Y(Kp="<<pid_y.kp<<", Ki="<<pid_y.ki<<")"<<endl;
    /***********************************************************
    *testes
    ***********************************************************/
    pid px,py;
    px.kp = 0.5;
    px.ki = 0.1;
    px.set_limits(0.0,(float)cap.get(CV_CAP_PROP_FRAME_WIDTH));
    px.setpoint = (float)cap.get(CV_CAP_PROP_FRAME_WIDTH)/2;
    px.v_init = (float)cap.get(CV_CAP_PROP_FRAME_WIDTH)/2;
    py.kp = 0.5;
    py.ki = 0.1;
    py.set_limits(0.0,(float)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    py.setpoint = (float)cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2;
    py.v_init = (float)cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2;
    /***********************************************************/
    //    cap.set(CV_CAP_PROP_GAIN, 48);
//    cap.set(CV_CAP_PROP_BRIGHTNESS, 10);

    namedWindow("window",1);
    setMouseCallback("window", CallBackFunc, NULL);

    Mat frame,aux_hsv,result;

    vector<Mat> HSV_chanells;

    vector<vector<Point> > contours;
    vector<Point> approx;

    bool obj_detected = false;
    int key;

    float out_x=pid_x.v_init;
    float out_y=pid_y.v_init;
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
        erode( frame, frame, element );
        //dilate( frame, frame, element );
        #endif
        /********************************************************************************************************************************/

        inRange(aux_hsv,Scalar(color[0]-FAIXA_H,color[1]-FAIXA_S,color[2]-FAIXA_V),Scalar(color[0]+FAIXA_H,color[1]+FAIXA_S,color[2]+FAIXA_V),result);
        Mat aux;
        result.copyTo(aux);
        findContours(result,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
//        if(contours.size()>0)
//            cout<<contours.size()<<" blob encontrado"<<endl;

        //            count_frames_without_blob=0;
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
        //cout << "Área: "<< blob_area <<endl;
        approxPolyDP(contours[blob_index], approx, arcLength(Mat(contours[blob_index]), true)*0.02, true);
        /************************************************************
         * Extração do centro do blob
         ************************************************************/
        //Point2f center;                                     ///Variavel que recebe o centro do blob.
        float radius;                                           ///Variavel auxiliar.
        minEnclosingCircle(approx,center,radius);           ///Acha o centro do blob.
        //circle( result, center, (int)radius, 255/*cv::Scalar(255,200,100)*/, 1);
        circle( frame, center, (int)radius, Scalar(255,0,0), 2);      ///Circulo que marca o blob.

        px.setpoint = center.x;
        py.setpoint = center.y;
        p_x = px.compute(p_x);
        p_y = py.compute(p_y);

        bool flag_pid = false;

        if(flag_pid)
        {
            out_x = pid_x.compute(p_x);
            out_y = pid_y.compute(p_y);
        }
        else
        {
            out_x = pid_x.compute(center.x);
            out_y = pid_y.compute(center.y);
        }

//        cout<<"setpoint: "<< (int)pid_x.setpoint <<" / Input: "<<(int)pid_x.input<<" / Erro: "<<(int)pid_x.error<<" / Saida: "<<(int)out_x<<endl;
        cout <<(int)(unsigned char)(int)out_x<<"   "<<(int)(unsigned char)(int)out_y<<endl;
//        int target_dist = _map(blob_area,300,3000,180,0);

        #ifdef _RASPI_
        serialPutchar(device,(unsigned char)200);
        serialPutchar(device,(unsigned char)(int)out_x);
        serialPutchar(device,(unsigned char)201);
        serialPutchar(device,(unsigned char)(int)out_y);
//        serialPutchar(device,(unsigned char)202);
//        serialPutchar(device,(unsigned char)target_dist);
        #endif // _RASPI_
        }
        else
        {
            px.setpoint = center_screen.x;
            py.setpoint = center_screen.y;
            p_x = px.compute(p_x);
            p_y = py.compute(p_y);
        }

//        circle(frame,Point(_x,_y),1,Scalar(100,255,50),3,2);        ///Circulo que marca o ponto clicado.
//        if(center.x >center_screen.x)line(frame,center_screen,center,Scalar(0,255,0),2);
//        else line(frame,center_screen,center,Scalar(0,0,255),2);
        #ifdef _RASPI_
        //imshow("result",aux);
        #endif // _RASPI
        circle(frame,center_screen,1,Scalar(10,255,50),3,2);        ///Circulo qua marca o centro.
        circle(frame,Point((int)p_x,(int)p_y),5,Scalar(255,255,100),3,2);///circulo que busca o blob
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

