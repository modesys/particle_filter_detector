#include "keller_reader.h"

Keller_reader::Keller_reader(std::string filename):
    keller_reader(filename)
{
    msgNumKeller = 0;
    keller_reader.read_header(io::ignore_extra_column,
                              "timestamp", "depth", "temp","pressure");
}

bool Keller_reader::nextLine()
{
    if(keller_reader.read_row(keller_data.timestamp, keller_data.depth,
                              keller_data.temperature, keller_data.pressure))
    {
//        packPressureMsg();
//        packCustom_TemperatureMsg();
//        packCustom_DepthMsg();
        packDepthMsg();
        packTemperatureMsg();
        packPressure();
        msgNumKeller++;
        return true;
    }
    else
    {
        return false;
    }
}

void Keller_reader::packDepthMsg()
{
    timestampToDouble_t currentTime = (keller_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    depthMessage.header.stamp = stamp;
    depthMessage.header.frame_id = "fluid_pressure";
    depthMessage.header.seq = msgNumKeller;
    depthMessage.fluid_pressure = keller_data.depth;
}

void Keller_reader::packTemperatureMsg()
{
    timestampToDouble_t currentTime = (keller_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    temperatureMessage.header.stamp = stamp;
    temperatureMessage.header.frame_id = "fluid_pressure";
    temperatureMessage.header.seq = msgNumKeller;
    temperatureMessage.fluid_pressure = keller_data.temperature;
}

void Keller_reader::packPressure()
{
    timestampToDouble_t currentTime = (keller_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    pressurehMessage.header.stamp = stamp;
    pressurehMessage.header.frame_id = "fluid_pressure";
    pressurehMessage.header.seq = msgNumKeller;
    pressurehMessage.fluid_pressure = keller_data.pressure;
}




//void Keller_reader::packPressureMsg()
//{
//    timestampToDouble_t currentTime = (keller_data.timestamp)/1e6;
//    ros::Time stamp(currentTime);
//    pMsg.header.stamp = stamp;
//    pMsg.header.frame_id = "fluid_pressure";
//    pMsg.header.seq = msgNumKeller;
//    pMsg.fluid_pressure = keller_data.pressure;
//}

//void Keller_reader::packCustom_TemperatureMsg()
//{
//    double doubleTime = double(keller_data.timestamp)/1e6;
//    ros::Time stamp(doubleTime);
//    tMsg.header.stamp = stamp;
//    tMsg.header.frame_id = "fluid_pressure";
//    tMsg.header.seq = msgNumKeller;
//    tMsg.temeperature = keller_data.temperature;
//}

//void Keller_reader::packCustom_DepthMsg()
//{
//    double doubleTime = double(keller_data.timestamp)/1e6;
//    ros::Time stamp(doubleTime);
//    dMsg.header.stamp = stamp;
//    dMsg.header.frame_id = "fluid_pressure";
//    dMsg.header.seq = msgNumKeller;
//    dMsg.depth = keller_data.depth;
//}


