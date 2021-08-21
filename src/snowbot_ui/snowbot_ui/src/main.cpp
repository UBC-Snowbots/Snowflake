/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbot UI
 */

#include <QtGui>
#include <QApplication>
#include "ros/ros.h"
#include "mainwindow.h"


int main(int argc, char **argv) {
    QApplication a(argc, argv);
    ros::init(argc, argv, "snowbot_ui");

    MainWindow w;
    w.show();

    qDebug() << "main";

    return a.exec();

}

