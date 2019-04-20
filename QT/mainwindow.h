#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>

#include <vector>
#include "ImageStitching/ImageStitching.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_openPic1_clicked();
    void on_openPic2_clicked();
    //void slot_ScroolWidget(int);

    void on_showButton1_clicked();

    void on_showButton2_clicked();

    void on_showButton3_clicked();

	void on_runButton_clicked();

	void on_a1_clicked();

	void on_a2_clicked();

	void on_a3_clicked();

	void on_a4_clicked();

private:
    Ui::MainWindow *ui;
    //QString pics[5];
	std::vector<QString> pics;

	ImageStitcher stitcher;
};

#endif // MAINWINDOW_H
