#-------------------------------------------------
#
# Project created by QtCreator 2015-06-08T16:51:49
#
#-------------------------------------------------

QT       += core

QT       -= gui

QT += serialport

TARGET = Teste_opencv_serial_Qt
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

win32:INCLUDEPATH += D:\Opencv\Release\install\include
win32:LIBS += D:\Opencv\Release\bin\libopencv_calib3d249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_contrib249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_core249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_features2d249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_flann249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_gpu249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_highgui249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_imgproc249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_legacy249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_ml249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_nonfree249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_objdetect249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_ocl249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_photo249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_stitching249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_superres249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_stitching249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_video249.dll
win32:LIBS += D:\Opencv\Release\bin\libopencv_videostab249.dll

unix:INCLUDEPATH += /usr/include
unix:INCLUDEPATH += -I/usr/X11/include/X11

unix:LIBS += -L/usr/lib\
-lopencv_core\
-lopencv_imgproc\
-lopencv_highgui\
-lopencv_ml\
-lopencv_video\
-lopencv_features2d\
-lopencv_calib3d\
-lopencv_objdetect\
-lopencv_contrib\
-lopencv_legacy\
-lopencv_flann\
