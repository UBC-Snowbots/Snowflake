/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

//QT
#include "QMessageBox"
#include "QDebug"
#include "QProcess"
#include "QLabel"
#include "QPixmap"
#include "QDir"

MainWindow::MainWindow(QWidget *parent) :

    QMainWindow(parent),
    ui (new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Snowbots Interface");
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()),this,SLOT(twist_values()));;
    timer->start(500);

    //ROS
    ros_f = new _Ros();
    qDebug() << "Constructor OK";

    //UI

    //Snowbots Logo
    QPixmap pixmap("./src/snowbots_ui/resources/snowbot2.png");
    ui->label_5->setPixmap(pixmap);
    ui->label_5->show();
    ui->label_5->setScaledContents(true);
    qDebug() << "Current dir:" << QDir::currentPath();

}
MainWindow::~MainWindow()
{
    delete ui;
    qDebug() << "Destructor OK";
}

void MainWindow::twist_values()
{
    ui->left_lcd->display(twist_message_left.linear.x);
    ui->right_lcd->display(twist_message_right.linear.x);
    ui->angular_lcd->display(twist_message_controller.angular.z);
    ui->linear_lcd->display(twist_message_controller.linear.x);
    ros_f->twist_subscriber();
}


