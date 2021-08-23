/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

// ros
#include "RosIntegration.h"
#include "ros/ros.h"
namespace Ui {
class MainWindow;
}
class MainWindow : public QMainWindow {
    Q_OBJECT
  public:
    explicit MainWindow(QWidget* parent = nullptr);
    virtual ~MainWindow();

  public Q_SLOTS:
    void twist_values();

    /*/private slots: Note if you want to create function from mainwindow.ui
     * delete
     * private slots and put them with public QSLOTS
     * */

  private:
    Ui::MainWindow* ui;
    RosIntegration* ros_f;
    QTimer* timer;
};

#endif // MAINWINDOW_H
