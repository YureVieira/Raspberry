#include <iostream>
#include <sstream>
#include <math.h>
#include <string.h>
#include <opencv.hpp>
//#define ERODE
//#define PI 3.14159265

int FAIXA_H =5;
int FAIXA_S =25;
int FAIXA_V =42;
int limiar = 80;
int cut_vertical=0;
float konst = 0.3;
float k_blob = 100000;
float k_y = 0.1;

using namespace std;
using namespace cv;

Mat image_HSV;
int _x,_y;
int v1=20,v2=90,N=11;
Vec3b color;
Point center_screen;
Point2f center;         ///Variavel que recebe o centro do blob.
bool flag_color = true;
/************************************************************************************************/
///trigonometrica(testes)
int img_height = 480;
int img_width = 640;
///Classe para conversão
class converter_coor
{
  float a;
  float b;
  float c;
  float f;
public:
  float Px = 0, Py = 0;
  void transform_coor(float d, Point p);
  void show_result();
}
transform_XY;

void converter_coor::transform_coor(float d,Point p)
{
    cout<<d<<endl;
    float a= d/cos(37.5*M_PI/180);
    float b = a*cos(75*M_PI/180);
    float c = a*sin(75*M_PI/180);
    float f=a*tan(52.5*M_PI/180);
    Px= (((a-b)*p.x/img_width)-a)*(-1);
    Py= p.x*c/img_width;
}
void converter_coor::show_result()
{
    Mat draws = Mat(img_height,img_width,CV_8UC3,Scalar(0,0,0));
    line(draws,Point(int(a),0),Point(0,int(f)),Scalar(0,0,200),2);
    line(draws,Point(0,0),Point(int(img_height*tan(15*M_PI/180)),img_height),Scalar(0,200,0),2);
    line(draws,Point(0,0),Point(int(Px),int(Py)),Scalar(100,255,255),2);
    circle(draws,Point(int(Px),int(Py)),3,Scalar(127,100,0),3);
    imshow("Test",draws);
}
/************************************************************************************************/
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
    cout << "color_HSV: [" << (int)color[0]<< ","<< (int)color[1]<< ","<< (int)color[2]<<"]" << endl;
  }
  else if ( event == EVENT_MOUSEMOVE && flag_Release)
  {
    _x=x;
    _y=y;
  }
}
//void show_xy_color(Mat img,int x,int y)
//{
//  Vec3b color = img.at<Vec3b>(Point(x,y));
//  Mat M(200,200,CV_8UC3,Scalar((int)color[0],255,100));
//  cvtColor(M,M,CV_HSV2BGR);
//  //imshow("Color",M);
//  cout << "color_HSV: [" << (int)color[0]<< ","<< (int)color[1]<< ","<< (int)color[2]<<"]" << endl;
//}

void show_dist(float Z,float K,float X)
{
  Mat M(500,500,CV_8UC3,Scalar(0,0,0));
  //Calculos
  Z = Z*K;
  double angle = asin (X/Z);
  double x = Z*sin(angle+M_PI/4);
  double y = Z*cos(angle+M_PI/4);
  cout<<"("<<x<<", "<<y<<")angle= "<<angle<<endl;
  int z = (int)Z;
  line(M,Point(0,0),Point((int)x,(int)y),Scalar(0,255,255),2);
  circle(M,Point((int)x,(int)y),5,Scalar(255,255,0),3);
  imshow("graph",M);
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
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,img_height);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,img_width);
  //    img_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  //    img_width = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  center_screen = Point(cap.get(CV_CAP_PROP_FRAME_WIDTH)/2,cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);
  color[0] = 103;         ///Azul
  //    cap.set(CV_CAP_PROP_GAIN, 48);
  //    cap.set(CV_CAP_PROP_BRIGHTNESS, 10);

  namedWindow("window",1);
  namedWindow("result",1);
  createTrackbar("H","window", &FAIXA_H, 90);
  createTrackbar("S","window", &FAIXA_S, 126);
  createTrackbar("V","window", &FAIXA_V, 126);
  createTrackbar("Limiar","result", &limiar, 255);
  createTrackbar("cut","result", &cut_vertical, cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2-1);
  setMouseCallback("window", CallBackFunc, NULL);


  Mat frame0,pre_img,result,aux;

  vector<Mat> HSV_chanells;

  vector<vector<Point> > contours;
  vector<Point> approx;

  for(;;)
  {
    cap >> frame0;
    //        GaussianBlur(frame, frame, Size(5, 5), 2, 1 );//aplique um filtro

    //Corte na imagem
    Mat frame = Mat(frame0, Rect(0,//Imagem cortada
    cut_vertical,
    cap.get(CV_CAP_PROP_FRAME_WIDTH),
    cap.get(CV_CAP_PROP_FRAME_HEIGHT) - cut_vertical*2));

///    imshow("subframe",frame);
    if(flag_color)
    {   //Imagens coloridas(segmentação de cores)
      medianBlur(frame,frame,5);
      cvtColor(frame,pre_img,CV_BGR2HSV);
      pre_img.copyTo(image_HSV);
    }
    else
    {   //Tons de cinza(segmentação simples,lousa)
      GaussianBlur(frame, frame, cv::Size(5, 5), 2, 2 );  //aplique um filtro
      cvtColor(frame,pre_img,CV_BGR2GRAY);               //Converte para tons de cinza e guarda em img_gray.
      threshold(pre_img,                                 //Imagem de origem.
      pre_img,                                 //Image de destino.
      limiar,                                      //Limiar.
      255,                                      //
      CV_THRESH_BINARY);                        //

      pre_img.copyTo(aux);

      Canny(pre_img, result, v1, v2);                 //Busca de bordas.

      findContours(result,
      contours,
      CV_RETR_LIST,
      CV_CHAIN_APPROX_SIMPLE);
    }


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
    if(flag_color)
    {
      inRange(pre_img,Scalar(color[0]-FAIXA_H,color[1]-FAIXA_S,color[2]-FAIXA_V),Scalar(color[0]+FAIXA_H,color[1]+FAIXA_S,color[2]+FAIXA_V),result);
      result.copyTo(aux);
      findContours(result,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    }
    /************************************************************
     * Segmentação dos blobs
     ***********************************************************/
    int blob_index=0;
    int blob_area=0;
    int area_min;
    if(flag_color)area_min = 100;
    else area_min = 10;
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
      Rect boundRect = boundingRect( Mat(approx) );
      rectangle( frame, boundRect.tl(), boundRect.br(), Scalar(126,200,100), 2);
      /************************************************************
       * Extração do centro do blob
       ************************************************************/
      //Point2f center;                                     ///Variavel que recebe o centro do blob.
      float radius;                                           ///Variavel auxiliar.
      minEnclosingCircle(approx,center,radius);           ///Acha o centro do blob.
//      transform_XY.transform_coor((k_blob/blob_area),center);
        transform_XY.transform_coor((img_height - center.y),center);
        transform_XY.show_result();

      //circle( result, center, (int)radius, 255/*cv::Scalar(255,200,100)*/, 1);
      //        circle( frame, center, (int)radius, cv::Scalar(255,0,0), 2);      ///Circulo que marca o blob.
      //        show_dist((float)blob_area,konst,center.x);
      //        cout <<"centro:("<< (int)center.x<<", "<<(int)center.y<<") / Area="<<blob_area<< " / Raio="<<radius<<endl;
    }

    //        circle(frame,center_screen,5,Scalar(10,255,50),3,2);        ///Circulo qua marca o centro.
    circle(frame,Point(_x,_y),1,Scalar(100,255,50),3,2);        ///Circulo que marca o ponto clicado.

    //        if(center.x >center_screen.x)line(frame,center_screen,center,Scalar(0,255,0),2);
    //        else line(frame,center_screen,center,Scalar(0,0,255),2);

    Rect rect_aux = Rect(0,
    cut_vertical,
    cap.get(CV_CAP_PROP_FRAME_WIDTH),
    cap.get(CV_CAP_PROP_FRAME_HEIGHT) - cut_vertical*2);

    rectangle( frame0, rect_aux.tl(), rect_aux.br(), Scalar(0,0,255), 1);

    imshow("result",aux);
    imshow( "window", frame0 );

    int key = waitKey(10);
    if(key==(int)'c')flag_color = !flag_color;
    if(key==(int)'q')break;     ///Sair com letra q

  }
  destroyAllWindows();
  return 0;
}
/******************************************************************************/

/******************************************************************************/


