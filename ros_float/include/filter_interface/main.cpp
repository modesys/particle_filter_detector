#include <ros/ros.h>
#include "particlefiltermaingui.h"
#include <qt5/QtWidgets/QApplication>
#include <qt5/QtCore/qglobal.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "particlefiltermaingui_node");
    QApplication a(argc, argv);
    particleFilterMainGUI w;
    w.show();

    return a.exec();
}


