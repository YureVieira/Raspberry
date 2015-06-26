#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serialPort.setPortName("COM5");         //Nome da porta
    serialPort.setBaudRate(9600);           //Baud rate

    //Tentar abrir para escrita somente
    if (!serialPort.open(QIODevice::WriteOnly)) {
        cout<<"Deu ruim"<<endl;
        close();
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    serialPort.write((char*)202);
    serialPort.write((char*)1);
}

void MainWindow::on_pushButton_2_clicked()
{
    serialPort.write((char*)202);
    serialPort.write((char*)2);
}

void MainWindow::on_pushButton_3_clicked()
{
    serialPort.write((char*)202);
    serialPort.write((char*)0);
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    serialPort.write((char*)200);
    serialPort.write((char*)value);
}
