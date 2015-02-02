#include <iostream>
#include <sstream>
#include <math.h>
#include <string.h>
#include <opencv.hpp>
//#define _RASPI_

#ifdef _RASPI_
#include <wiringSerial.h>
#endif // _RASPI_

#define FAIXA_H 5
#define FAIXA_S 25
#define FAIXA_V 5

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
char dev1[]="/dev/ttyUSB0",dev2[]="/dev/ttyACM0";
/******************************************************************************/
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
      output = kp * error + Iterm;
    if(output > outMax) output = outMax;
    else if(output < outMin) output = outMin;
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
    color[0] = 103;         ///Azul
    pid pid_x;
    pid_x.kp = 1;
    pid_x.ki = 0.3;
    pid_x.set_limits(-255.0,255.0);
    pid_x.setpoint = 160.0;
#ifdef _RASPI_
    device = serialOpen(dev2, baud_rate);
#endif // _RASPI_
    VideoCapture cap(0);
    if(!cap.isOpened()) // check if we succeeded
        return 1;

    cap.set(CV_CAP_PROP_FRAME_HEIGHT,320);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,240);
    center_screen = Point(cap.get(CV_CAP_PROP_FRAME_WIDTH)/2,cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);
    //    cap.set(CV_CAP_PROP_GAIN, 48);
//    cap.set(CV_CAP_PROP_BRIGHTNESS, 10);

    namedWindow("window",1);
    setMouseCallback("window", CallBackFunc, NULL);

    Mat frame,aux_hsv,result;
    vector<Mat> HSV_chanells;

    vector<vector<Point> > contours;
    vector<Point> approx;

    for(;;)
    {
        cap >> frame;
        //GaussianBlur(frame, frame, Size(5, 5), 2, 1 );//aplique um filtro
        cvtColor(frame,aux_hsv,CV_BGR2HSV);
        aux_hsv.copyTo(image_HSV);

        /********************************************************************************************************************************/
        ///Erosão
        int erosion_size = 3;
        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                             Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                             Point( erosion_size, erosion_size ) );
        /// Apply the erosion operation
        //erode( result, result, element );
        erode( frame, frame, element );
        //dilate( frame, frame, element );
        /********************************************************************************************************************************/

        //Separa canais para verificar somente Matiz.
        Mat aux(frame.size(), CV_8U);
        split(aux_hsv,HSV_chanells);

        threshold(HSV_chanells[0],HSV_chanells[0],min(179,(int)color[0]+FAIXA_H),255,THRESH_TOZERO_INV);
        threshold(HSV_chanells[0],HSV_chanells[0],max(0,(int)color[0]-FAIXA_H),255,THRESH_TOZERO);
        threshold(HSV_chanells[0],result,1,255,THRESH_BINARY);
//        imshow("1",HSV_chanells[0]);
        //result.copyTo(aux);

//        threshold(HSV_chanells[1],HSV_chanells[1],min(255,(int)color[1]+FAIXA_S),255,THRESH_TOZERO_INV);
//        threshold(HSV_chanells[1],HSV_chanells[1],max(0,(int)color[1]-FAIXA_S),255,THRESH_TOZERO);
//        threshold(HSV_chanells[1],HSV_chanells[1],1,255,THRESH_BINARY);
//        imshow("2",HSV_chanells[1]);
//        //bitwise_not(HSV_chanells[1],HSV_chanells[1]);
////
//        threshold(HSV_chanells[2],HSV_chanells[2],100,255,THRESH_TOZERO_INV);
//        threshold(HSV_chanells[2],HSV_chanells[2],20,255,THRESH_TOZERO);
//        threshold(HSV_chanells[2],HSV_chanells[2],1,255,THRESH_BINARY);
//        //bitwise_not(HSV_chanells[2],HSV_chanells[2]);
//        imshow("3",HSV_chanells[2]);
////
//        bitwise_and(HSV_chanells[0],HSV_chanells[1],result);
//        bitwise_and(result,HSV_chanells[2],result);

        //Junta novamete os canais.
        //merge(HSV_chanells,aux_hsv);
        //cvtColor(aux_hsv,result,CV_HSV2BGR);

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
        int area_min = 300;
        /***********************************************************
         * Separa o maior blob para segmentação
        ***********************************************************/
        for( size_t i = 0; i < contours.size() ; i++ )
        {
            if(fabs(contourArea(Mat(contours[i]))) > area_min /*&& fabs(contourArea(Mat(contours[i]))) < 10000*/)
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
        circle( frame, center, (int)radius, cv::Scalar(5,5,255), 2);      ///Circulo que marca o blob.
        float out = pid_x.compute(center.x);
        cout<<"Input: "<<pid_x.input<<" / Erro: "<<pid_x.error<<" / Saida: "<<out<<endl;
        }



#ifdef _RASPI_
        /***********************************************************
        Prepara e envia coordenas pela serial
        ************************************************************/
        error_x = (short)center_screen.x - (short)center.x;
        error_y = (short)center_screen.y - (short)center.y;
        info_x._int=error_x;
        info_y._int=error_y;
        char message[]= {'a',info_x._char[0],info_x._char[1],info_y._char[0],info_y._char[1],'z',0};
        cout<<"X: "<<error_x<<",Y: "<<error_y<<endl;
        for(int i=0; i<6; i++)message[6]+=message[i];
        for(int i=0; i<7; i++)serialPutchar(device, message[i]);
#endif // _RASPI_
        circle(frame,center_screen,5,Scalar(10,255,50),3,2);        ///Circulo qua marca o centro.
        circle(frame,Point(_x,_y),1,Scalar(100,255,50),3,2);        ///Circulo que marca o ponto clicado.
        //imshow("result",result);
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

