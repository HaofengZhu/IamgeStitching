#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QScrollArea>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowState(Qt::WindowMaximized);//窗体默认最大化

    //设置各个控件的大小和位置
    ui->openPic1->setGeometry(70,170,150,50);
    ui->openPic2->setGeometry(70,520,150,50);
    //ui->res->setGeometry(70,880,150,50);
	ui->res->setGeometry(950, 600, 150, 50);

	ui->runButton->setGeometry(920, 230, 150, 40);

    ui->pic1->setGeometry(250,50,600,300);
    ui->pic2->setGeometry(250,400,600,300);
    ui->resPic->setGeometry(1300,600,600,300);

    ui->choose->setGeometry(900,60,150,40);
    ui->a1->setGeometry(900,110,80,20);
    ui->a2->setGeometry(1000,110,80,20);
    ui->a3->setGeometry(1100,110,80,20);
    ui->a4->setGeometry(1200,110,80,20);


    ui->show->setGeometry(900,360,200,40);
    ui->showButton1->setGeometry(920,420,250,40);
    ui->showButton2->setGeometry(920,470,250,40);
    ui->showButton3->setGeometry(920,520,250,40);
    ui->interPic->setGeometry(1300,360,1200,300);

}

MainWindow::~MainWindow()
{
    delete ui;

}

void MainWindow::on_openPic1_clicked()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Open Image"), ".",
                                                tr("Image Files(*.jpg *.png)"));
    if(path.length() == 0) {
        QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files."));
    } else {
        //QMessageBox::information(NULL, tr("Path"), tr("You selected ") + path);
        QImage image;
        image.load(path);//fileName为图片的路径
        QImage result = image.scaled(600,300,Qt::KeepAspectRatio,
                                     Qt::SmoothTransformation);//放缩图片，以固定大小显示

        ui->pic1->setPixmap(QPixmap::fromImage(result));

		stitcher.setLeftImagePath(path.toStdString());
    }
}

void MainWindow::on_openPic2_clicked()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Open Image"), ".",
                                                tr("Image Files(*.jpg *.png)"));
    if(path.length() == 0) {
        QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files."));
    } else {
        //QMessageBox::information(NULL, tr("Path"), tr("You selected ") + path);
        //filename1=path;
        QImage image;
        image.load(path);//fileName为图片的路径
        QImage result = image.scaled(600,300,Qt::KeepAspectRatio,
                                     Qt::SmoothTransformation);//放缩图片，以固定大小显示

        ui->pic2->setPixmap(QPixmap::fromImage(result));

		stitcher.setRightImagePath(path.toStdString());
    }
}

void MainWindow::on_showButton1_clicked()
{
    QImage image;
    image.load(pics[1]);//fileName为图片的路径
    QImage result = image.scaled(600,300,Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);//放缩图片，以固定大小显示
    ui->interPic->setPixmap(QPixmap::fromImage(result));
}

void MainWindow::on_showButton2_clicked()
{
    QImage image;
    image.load(pics[2]);//fileName为图片的路径
    QImage result = image.scaled(600,300,Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);//放缩图片，以固定大小显示
    ui->interPic->setPixmap(QPixmap::fromImage(result));
}

void MainWindow::on_showButton3_clicked()
{
    QImage image;
    image.load(pics[3]);//fileName为图片的路径
    QImage result = image.scaled(600,300,Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);//放缩图片，以固定大小显示
    ui->interPic->setPixmap(QPixmap::fromImage(result));
}

void MainWindow::on_runButton_clicked()
{
	try {
		ui->resPic->setText("拼接中...");
		stitcher.loadImages();
		stitcher.runPipeline();
		vector<string> picsPath = stitcher.saveImages();
		for (auto it : picsPath) pics.push_back(QString::fromStdString(it));
	}
	catch (exception& ex) {
		QMessageBox::information(NULL, "Error", ex.what());
		return;
	}

	//在“中间结果显示”处显示两张图片相邻摆放的结果
   QImage image;
   image.load(pics[0]);
   QImage result = image.scaled(600,300,Qt::KeepAspectRatio,
                                Qt::SmoothTransformation);//放缩图片，以固定大小显示
   ui->interPic->setPixmap(QPixmap::fromImage(result));

   //在“最终结果”处显示最终结果
   QImage image1;
   image1.load(pics[4]);
   QImage result1 = image1.scaled(600,300,Qt::KeepAspectRatio,
                                Qt::SmoothTransformation);//放缩图片，以固定大小显示
   ui->resPic->setPixmap(QPixmap::fromImage(result1));
}

void MainWindow::on_a1_clicked()
{
	stitcher.setFeatureExtractor(Feature::SIFT);
}

void MainWindow::on_a2_clicked()
{
	stitcher.setFeatureExtractor(Feature::SURF);
}

void MainWindow::on_a3_clicked()
{
	stitcher.setFeatureExtractor(Feature::ORB);
}

void MainWindow::on_a4_clicked()
{
	stitcher.setFeatureExtractor(Feature::FERNS);
}
