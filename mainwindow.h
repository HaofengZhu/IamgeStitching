#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>

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

private:
    Ui::MainWindow *ui;
    QString pics[5] = {"C://Users//kunxi//Desktop//0.png","C://Users//kunxi//Desktop//1.jpg","C://Users//kunxi//Desktop//2.jpg","C://Users//kunxi//Desktop//3.jpg","C://Users//kunxi//Desktop//4.jpg"};

};

#endif // MAINWINDOW_H
