#include "mainwindow.h"
#include <QApplication>
#include <QProcess>
#include <QMessageBox>
//#include <opencv2/core/core.hpp>
//#include <opencv2/xfeatures2d/nonfree.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/imgproc/imgproc.hpp"


int main(int argc, char *argv[])
{
    //cv::xfeatures2d::SiftFeatureDetector detector;

    ////cv::Mat image01=cv::imread("C:/Users/kunxi/Desktop/4.jpg");
    ////cv::imshow("picture", image01);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
