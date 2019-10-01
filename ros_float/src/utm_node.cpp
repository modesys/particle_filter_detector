#include <ros/ros.h>
#include "../include/ros_float/zone_converter.h"
//#include <qt5/QtWidgets/QApplication>
//#include <qt5/QtCore/qglobal.h>

int main(int argc, char *argv[])
{
    (void) argc;
    (void) argv;
    ros::init(argc, argv, "utm_node");

    ZONE_converter *convert;
    double lat = 41.4484;
    double lon =  -71.3999;

    double UTMNorthing;
    double UTMEasting;
    std::string UTMZone;

    convert->UTM(lat, lon, UTMNorthing, UTMEasting);
    //convert->fromLatLontoUTM(lat, lon, UTMNorthing, UTMEasting, UTMZone);
    //convert->allOtherZones(lat);
    std::cout<<lat <<" "<<lon<<" "<<UTMEasting<<" "<<UTMNorthing<<" "<<std::stoi(UTMZone) << std::endl;

//    QApplication a(argc, argv);
//    UTM_Zone w;
//    w.show();
//    return a.exec();
}


//int i;
//    std::cout<<"====== LATITUDE & LONGITUDE INTO UTM ====== "<<"\n";
//    std::cout<<"--------------------------------------------"<<"\n";
//    std::cout<<"Enter latitude ::     "<<"\n";
//    std::cin >> i;
//    std::cout<<"Enter longitude ::    "<<"\n";
//    std::cin >> i;
//    std::cout<<"Enter UTMNorthing ::  "<<"\n";
//    std::cin >> i;
//    std::cout<<"Enter UTMEasting ::   "<<"\n";
//    std::cin >> i;
//    std::cout<<"Enter UTMZone ::      "<<"\n";
//    std::cin >> i;
//    std::cout<<"\n";

//    ZONE_converter *convert;
//    double lat = 26.28174;
//    double lon =  92.14268;

//    double UTMNorthing;
//    double UTMEasting;
//    std::string UTMZone;

//  //  convert->UTM(lat, lon, eastingUtmzone, northingUtmzone);
//    convert->fromLatLontoUTM(lat, lon, UTMNorthing, UTMEasting, UTMZone);
//    //convert->allOtherZones(lat);
//    std::cout<<lat <<" "<<lon<<" "<<UTMEasting<<" "<<UTMNorthing<<" "<<std::stoi(UTMZone) << std::endl;

