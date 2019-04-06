/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *pic1;
    QPushButton *openPic1;
    QPushButton *openPic2;
    QLabel *pic2;
    QLabel *resPic;
    QLabel *show;
    QPushButton *showButton1;
    QPushButton *showButton2;
    QPushButton *showButton3;
    QLabel *choose;
    QRadioButton *a4;
    QRadioButton *a3;
    QRadioButton *a1;
    QRadioButton *a2;
    QLabel *res;
    QLabel *interPic;
    QPushButton *runButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(949, 576);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        pic1 = new QLabel(centralWidget);
        pic1->setObjectName(QString::fromUtf8("pic1"));
        pic1->setGeometry(QRect(180, 50, 171, 141));
        openPic1 = new QPushButton(centralWidget);
        openPic1->setObjectName(QString::fromUtf8("openPic1"));
        openPic1->setGeometry(QRect(10, 90, 111, 31));
        openPic2 = new QPushButton(centralWidget);
        openPic2->setObjectName(QString::fromUtf8("openPic2"));
        openPic2->setGeometry(QRect(10, 290, 111, 31));
        pic2 = new QLabel(centralWidget);
        pic2->setObjectName(QString::fromUtf8("pic2"));
        pic2->setGeometry(QRect(180, 290, 181, 151));
        resPic = new QLabel(centralWidget);
        resPic->setObjectName(QString::fromUtf8("resPic"));
        resPic->setGeometry(QRect(770, 370, 71, 41));
        show = new QLabel(centralWidget);
        show->setObjectName(QString::fromUtf8("show"));
        show->setGeometry(QRect(370, 170, 171, 31));
        showButton1 = new QPushButton(centralWidget);
        showButton1->setObjectName(QString::fromUtf8("showButton1"));
        showButton1->setGeometry(QRect(370, 200, 161, 31));
        showButton2 = new QPushButton(centralWidget);
        showButton2->setObjectName(QString::fromUtf8("showButton2"));
        showButton2->setGeometry(QRect(370, 240, 161, 31));
        showButton3 = new QPushButton(centralWidget);
        showButton3->setObjectName(QString::fromUtf8("showButton3"));
        showButton3->setGeometry(QRect(370, 280, 161, 31));
        choose = new QLabel(centralWidget);
        choose->setObjectName(QString::fromUtf8("choose"));
        choose->setGeometry(QRect(371, 41, 95, 23));
        a4 = new QRadioButton(centralWidget);
        a4->setObjectName(QString::fromUtf8("a4"));
        a4->setGeometry(QRect(570, 70, 71, 21));
        a3 = new QRadioButton(centralWidget);
        a3->setObjectName(QString::fromUtf8("a3"));
        a3->setGeometry(QRect(500, 70, 51, 16));
        a1 = new QRadioButton(centralWidget);
        a1->setObjectName(QString::fromUtf8("a1"));
        a1->setGeometry(QRect(360, 70, 81, 16));
        a1->setChecked(true);
        a2 = new QRadioButton(centralWidget);
        a2->setObjectName(QString::fromUtf8("a2"));
        a2->setGeometry(QRect(430, 70, 71, 16));
        res = new QLabel(centralWidget);
        res->setObjectName(QString::fromUtf8("res"));
        res->setGeometry(QRect(410, 370, 91, 31));
        interPic = new QLabel(centralWidget);
        interPic->setObjectName(QString::fromUtf8("interPic"));
        interPic->setGeometry(QRect(770, 200, 91, 41));
        runButton = new QPushButton(centralWidget);
        runButton->setObjectName(QString::fromUtf8("runButton"));
        runButton->setGeometry(QRect(390, 120, 93, 28));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 949, 26));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        pic1->setText(QApplication::translate("MainWindow", "\345\233\276\347\211\207\344\270\200", nullptr));
        openPic1->setText(QApplication::translate("MainWindow", "\346\211\223\345\274\200\345\233\276\347\211\207\344\270\200...", nullptr));
        openPic2->setText(QApplication::translate("MainWindow", "\346\211\223\345\274\200\345\233\276\347\211\207\344\272\214...", nullptr));
        pic2->setText(QApplication::translate("MainWindow", "\345\233\276\347\211\207\344\272\214", nullptr));
        resPic->setText(QApplication::translate("MainWindow", "\346\213\274\346\216\245\345\233\276\347\211\207", nullptr));
        show->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">\344\270\255\351\227\264\347\273\223\346\236\234\346\230\276\347\244\272\357\274\232</span></p></body></html>", nullptr));
        showButton1->setText(QApplication::translate("MainWindow", "\346\240\207\346\263\250\347\211\271\345\276\201\347\202\271\345\220\216\347\232\204\347\273\223\346\236\234", nullptr));
        showButton2->setText(QApplication::translate("MainWindow", "\345\216\273\351\231\244outliner\345\220\216\347\232\204\347\273\223\346\236\234", nullptr));
        showButton3->setText(QApplication::translate("MainWindow", "\347\211\271\345\276\201\347\202\271\345\214\271\351\205\215\345\220\216\347\232\204\347\273\223\346\236\234", nullptr));
        choose->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">\351\200\211\346\213\251\347\256\227\346\263\225:</span></p></body></html>", nullptr));
        a4->setText(QApplication::translate("MainWindow", "FERNS", nullptr));
        a3->setText(QApplication::translate("MainWindow", "ORB", nullptr));
        a1->setText(QApplication::translate("MainWindow", "SIFT", nullptr));
        a2->setText(QApplication::translate("MainWindow", "SUFT", nullptr));
        res->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">\346\213\274\346\216\245\347\273\223\346\236\234\357\274\232</span></p></body></html>", nullptr));
        interPic->setText(QApplication::translate("MainWindow", "\344\270\255\351\227\264\347\273\223\346\236\234", nullptr));
        runButton->setText(QApplication::translate("MainWindow", "\350\277\220\350\241\214", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
