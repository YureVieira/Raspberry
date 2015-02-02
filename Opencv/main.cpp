#include "opencv2/opencv.hpp"
#include <iostream>
#include <sstream>
#include <math.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#define borda 1
#define A1 0
#define A2 1
#define B1 2
#define B2 3

using namespace std;
using namespace cv;

//int thresh = 50, N = 11;
int x_diff,y_diff;
int v1=20,v2=90,N=11;

const int baud_rate = 9600;
int device;
char dev1[]="/dev/ttyUSB0",dev2[]="/dev/ttyACM0";

void WritePkg(char d1,char d2)
{
    char data[4];
    data[0]=0xF1;
    data[3]=0xF2;
    data[1]=d1;
    data[2]=d2;
    serialPuts(device,data);
}
void frente()
{
	digitalWrite (A1, HIGH);
	digitalWrite (A2, LOW);
	digitalWrite (B1, LOW);
	digitalWrite (B2, HIGH);
}

void re()
{
	digitalWrite (A1, LOW);
	digitalWrite (A2, HIGH);
	digitalWrite (B1, HIGH);
	digitalWrite (B2, LOW);
}
void esquerda()
{
	digitalWrite (A1, LOW);
	digitalWrite (A2, HIGH);
	digitalWrite (B1, LOW);
	digitalWrite (B2, HIGH);
}
void direita()
{
	digitalWrite (A1, HIGH);
	digitalWrite (A2, LOW);
	digitalWrite (B1, HIGH);
	digitalWrite (B2, LOW);
}
void parar()
{
	digitalWrite (A1, LOW);
	digitalWrite (A2, LOW);
	digitalWrite (B1, LOW);
	digitalWrite (B2, LOW);
}
int main()
{
	/*********************************
	Configuração gpio
	**********************************/
	wiringPiSetup () ;
	pinMode (A1, OUTPUT) ;
	pinMode (A2, OUTPUT) ;
	pinMode (B1, OUTPUT) ;
	pinMode (B2, OUTPUT) ;
	/*********************************/
	device = serialOpen(dev2,baud_rate);
	/*********************************/
	char command = 0;
	//int device=serialOpen(dev1, baud_rate);
	//cout<<"dispositivo: "<<device<<endl;
	//if(device==-1)
	//{
		//serialClose(device);
		//device=serialOpen (dev2,baud_rate);
	//}
	//cout<<"dispositivo: "<<device<<endl;
	//if(device==-1)return 5;
	/*********************************
	Imagens de modelo
	**********************************/
	Mat frame,img_gray;
	vector<Mat> img_model;
	bool flag_compara_imagens = false,info_img_load[4];
	//Mat img_model = Mat::zeros(100,100,CV_8U);
	try
	{
		img_model.push_back(imread("Direita.jpg",CV_LOAD_IMAGE_GRAYSCALE));
		img_model.push_back(imread("Esquerda.jpg",CV_LOAD_IMAGE_GRAYSCALE));
		img_model.push_back(imread("Frente.jpg",CV_LOAD_IMAGE_GRAYSCALE));
		img_model.push_back(imread("Retorno.jpg",CV_LOAD_IMAGE_GRAYSCALE));
//		imshow("img1",img_model[0]);
//		imshow("img2",img_model[1]);
//		imshow("img3",img_model[2]);
//		imshow("img4",img_model[3]);
	}
	catch(Exception err)
	{
		cout<<err.msg<<endl;
	}
	//cout<<"Ola mundo"<<endl;
	/**********************************
	Verificação se todas as imagens
	foram carregadas.
	**********************************/
	for(int i=0;i<4;i++)
	{
		if(img_model[i].data)
		{
			cout<<"Imagem "<<i<<" carregada com sucesso!"<<endl;
			info_img_load[i]=true;
		}

		else
		{
			cout<<"Erro ao carregar a imagem: "<<i<<"."<<endl;
			info_img_load[i]=false;
		}

	}

	/***********************************
	Parametros para compressao
	das imagens que servirao de modelo
	************************************/
	vector<int> compression_param;
	compression_param.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_param.push_back(100);
	/**********************************/

	VideoCapture cap(0);
	if(!cap.isOpened())  // check if we succeeded
	return 1;
	//só para raspberry
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,320);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,240);
	Point cursor;
	//cursor.y=cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2;
	//cursor.x=cap.get(CV_CAP_PROP_FRAME_WIDTH)/2;

	namedWindow("window",1);

	vector<vector<Point> > squares;
	vector<vector<Point> > contours;
	vector<Point> approx;
	cout << "Para sair pressione 'q'." << endl;

	for(;;)
	{
		int key = waitKey(10);
		cap >> frame;//capture um frame
		//transpose(frame,frame);
		//flip(frame,frame,1);
		cursor.y = frame.rows/2;
		cursor.x=frame.cols/2;
		//GaussianBlur(frame, frame, Size(5, 5), 2, 2 );//aplique um filtro
		Mat img_canny(frame.size(), CV_8U);//nova matriz
		cvtColor(frame,img_gray,CV_BGR2GRAY);//passe a imagem para tons de cinza
		//split(img_HSV,img_channels);
		Canny(img_gray, img_canny, v1, v2);//aplique canny
		//imshow( "window3", img_canny);
		findContours(img_canny, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);//busque contornos

		//analise de cada contorno
		for( size_t i = 0; i < contours.size(); i++ )
		{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);//faça aproximações para polignos

			if( approx.size() == 4 &&//se é um quadrilatero...
			fabs(contourArea(Mat(approx))) > 500 &&//de area > que 500...
			isContourConvex(Mat(approx)) )//e é um controno fechado
			{
				command='a';
				//cout<<"Area: "<<fabs(contourArea(Mat(approx)))<<endl;
				/*********************************
				Achar o centro do contorno
				***********************************/
				Point2f center;
				int x=0,y=0;
				for(int a=0;a<4;a++)
				{
					x += approx[a].x;
					y += approx[a].y;
				}
				center.x = x/4;
				center.y = y/4;
				/*********************************
				Reorganiza os pontos de vertices
				***********************************/
				Point buffer[4];
				for(int a=0;a<4;a++)
				{
					if(approx[a].x <= center.x)
					{//ponto superior esquerdo
						if(approx[a].y <= center.y)
						{
							buffer[0].x = approx[a].x+borda;
							buffer[0].y = approx[a].y+borda;
						}
					}
					if(approx[a].x > center.x)
					{//ponto superior direito
						if(approx[a].y <= center.y)
						{
							buffer[1].x = approx[a].x-borda;
							buffer[1].y = approx[a].y+borda;
						}
					}
					if(approx[a].x > center.x)
					{//ponto inferior esquerdo
						if(approx[a].y > center.y)
						{
							buffer[2].x = approx[a].x-borda;
							buffer[2].y = approx[a].y-borda;
						}
					}
					if(approx[a].x <= center.x)
					{//ponto inferior direito
						if(approx[a].y > center.y)
						{
							buffer[3].x = approx[a].x+borda;
							buffer[3].y = approx[a].y-borda;
						}
					}
				}

				for(int a=0;a<4;a++)
				{
					approx[a] = buffer[a];
				}

				/*********************************
				Desenha linhas que liga os pontos
				***********************************/
				circle(frame,approx[0],2,Scalar(0,0,0),1,1);
				circle(frame,approx[1],2,Scalar(0,0,0),1,1);
				circle(frame,approx[2],2,Scalar(0,0,0),1,1);
				circle(frame,approx[3],2,Scalar(0,0,0),1,1);
				circle(frame,center,2,Scalar(0,0,0),-1,1);

				line(frame,cursor,center,Scalar(250,20,200),2,1);

				line(frame,approx[0],approx[1],Scalar(255,100,0),1,1);
				line(frame,approx[1],approx[2],Scalar(255,100,0),1,1);
				line(frame,approx[2],approx[3],Scalar(255,100,0),1,1);
				line(frame,approx[3],approx[0],Scalar(255,100,0),1,1);
				/*************************************************************************************************************/
                x_diff = (int)center.x - cursor.x;
                y_diff = (int)center.y - cursor.y;
                std::stringstream result_x,result_y;
                result_x << x_diff;
                result_y << y_diff;
				putText(frame,"X: "+result_x.str(),Point(center.x,center.y+10),FONT_HERSHEY_SIMPLEX,0.3,Scalar(250,100,220),1);
				putText(frame,"Y: "+result_y.str(),Point(center.x,center.y+30),FONT_HERSHEY_SIMPLEX,0.3,Scalar(250,100,220),1);
				//cout<<"X: "<< x_diff << endl;
				//cout<<"Y: "<< y_diff << endl;
				//cout<<"Area: "<< fabs(contourArea(Mat(approx))) << endl;
				//if(fabs(contourArea(Mat(approx))) < 7000)frente();
				//else if (y_dif >= 5) direita();
				if (x_diff >= 8) direita();
				else if (x_diff <= -8) esquerda();
				else parar();
				/*************************************************************************************************************/
				/**********************************
				Corrige pespectiva da imagem
				contida entre os pontos
				**********************************/
				vector<Point2f> pts;
				for(int a=0;a<4;a++)
				{
					pts.push_back(approx[a]);
				}
				Mat quad = Mat::zeros(120,120,CV_8U);
				vector<Point2f> corners;
				//adicione pontos ao vetor
				corners.push_back(Point2f(0,0));
				corners.push_back(Point2f(quad.cols,0));
				corners.push_back(Point2f(quad.cols,quad.rows));
				corners.push_back(Point2f(0,quad.rows));

				Mat transmtx = getPerspectiveTransform(pts,corners);
				warpPerspective(img_gray,quad,transmtx,quad.size());
				//warpPerspective(frame,quad,transmtx,quad.size());

				Mat img_binary;// = Mat::zeros(200,200,CV_8UC1);
				threshold(quad,img_binary,150,255,THRESH_BINARY);
				Mat img_resize = Mat::zeros(120,120,CV_8U);
				resize(img_binary,img_resize,Size(120,120));

				/**********************************
				Capturar e guardar a imagem
				qualificada como pertencente a
				um quadrilatero
				**********************************/
				//int key = waitKey(10);
				if(key == (int)'h')
				{
					flag_compara_imagens = true;
					img_model[0] = img_resize;
					cout<<"Gravado (esquerda)"<<endl;
					imwrite("Esquerda.jpg",img_model[0],compression_param);
				}
				if(key == (int)'k')
				{
					flag_compara_imagens = true;
					img_model[1] = img_resize;
					cout<<"Gravado (direita)"<<endl;
					imwrite("Direita.jpg",img_model[1],compression_param);
				}
				if(key == (int)'u')
				{
					flag_compara_imagens = true;
					img_model[2] = img_resize;
					cout<<"Gravado (em frente)"<<endl;
					imwrite("Frente.jpg",img_model[2],compression_param);
				}
				if(key == (int)'j')
				{
					flag_compara_imagens = true;
					img_model[3] = img_resize;
					cout<<"Gravado (voltar)"<<endl;
					imwrite("Retorno.jpg",img_model[3],compression_param);
				}
				key = 0;
				imshow("quadro", img_resize);

				/**********************************
				Comparar entre a imagem captada
				agora e a tomada como padrao
				**********************************/
				Mat result;
				for(int a=0;a<4;a++)
				{
					int count_non_zero=0;
					if(info_img_load[a])//se a imagem foi carregada com sucesso
					{
						bitwise_xor(img_resize, img_model[a], result);
						count_non_zero = countNonZero(result);
						imshow("result",result);
					}
					//cout<<"Ola mundo"<<endl;
					//imshow("result",result);

					//cout<<count_non_zero<<endl;
					if(count_non_zero < 1000 && info_img_load[a])
					{
						putText(frame,"Semelhantes",Point(0,10),FONT_HERSHEY_SIMPLEX,0.5,Scalar(100,255,80),2);
						if(a==0 && command != 'L')
						{
							cout<<"Esquerda"<<endl;
							putText(frame,"Esquerda",Point(0,40),FONT_HERSHEY_SIMPLEX,0.5,Scalar(100,255,80),2);
							esquerda();
							command ='L';
						}

						if(a==1 && command != 'R')
						{
							cout<<"Direita"<<endl;
							putText(frame,"Direita",Point(0,60),FONT_HERSHEY_SIMPLEX,0.5,Scalar(100,255,80),2);
							serialPutchar(device,'R');
							direita();
							command ='R';
						}
						if(a==2 && command != 'F')
						{
							cout<<"Frente"<<endl;
							putText(frame,"Frente",Point(0,80),FONT_HERSHEY_SIMPLEX,0.5,Scalar(100,255,80),2);
							serialPutchar(device,'F');
							frente();
							command ='F';
						}
						if(a==3 && command != 'B')
						{
							cout<<"Retorno"<<endl;
							putText(frame,"Retorno",Point(0,100),FONT_HERSHEY_SIMPLEX,0.5,Scalar(100,255,80),2);
							serialPutchar(device,'B');
							re();
							command ='B';
						}
					}
					else
					{
						//cout<<count_non_zero<<endl;
						putText(frame,"Diferentes",Point(0,230),FONT_HERSHEY_SIMPLEX,0.5,Scalar(100,255,80),2);
					}
				}
			}
			else
			{
				/*********************************
				brincadeira com pontos
				**********************************/
				//cursor.x=(cap.get(CV_CAP_PROP_FRAME_WIDTH)/2-cursor.x)/10;
				//cursor.y=(cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2-cursor.y)/10;
			}
		}

		/**********************************
		Leitura do teclado
		**********************************/
		circle(frame,cursor,2,Scalar(125,250,110),2);
		imshow( "window", frame );

		//int key = waitKey(10);
		if(key == (int)'q') break;
		else if(key == (int)'w') frente();
		else if(key == (int)'s') re();
		else if(key == (int)'d') direita();
		else if(key == (int)'a') esquerda();
		else parar();
	}
	serialClose(device);
	destroyAllWindows();
	cout << "Saindo" << endl;
	return 0;
}

