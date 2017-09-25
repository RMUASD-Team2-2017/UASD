#include <ros/ros.h>
#include "gcs/dockData.h"
#include "gcs/openDock.h"

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define DOCKING_STATION_CLASS docking_station


class DOCKING_STATION_CLASS
{
    public:
        DOCKING_STATION_CLASS(ros::NodeHandle *nh);
        ~DOCKING_STATION_CLASS();
        void publicData();
    private:
        ros::NodeHandle *nh;
        ros::Publisher pub;

        ros::ServiceServer openDock;
        bool openDockCallback(gcs::openDock::Request &req, gcs::openDock::Response &res);

        float getTemperature();
        float getHumidity();
        bool openDockingStation();

        // TODO Implement service monitorDock()

};

DOCKING_STATION_CLASS::DOCKING_STATION_CLASS(ros::NodeHandle *nh)
{
    this->nh = nh;

    // Services
    openDock = nh->advertiseService("/docking_station/openDock",&DOCKING_STATION_CLASS::openDockCallback, this); // Create openDock service

    // Topics
    pub = nh->advertise<gcs::dockData>("/docking_station/dock_data", 1000);
}

DOCKING_STATION_CLASS::~DOCKING_STATION_CLASS()
{
    delete nh;
}

bool DOCKING_STATION_CLASS::openDockCallback(gcs::openDock::Request &req, gcs::openDock::Response &res)
{
    res.dockIsOpen = openDockingStation();
    return true;
}

bool DOCKING_STATION_CLASS::openDockingStation()
{
    // Do some GPIO stuff here to open the docking station lastIndex
    return true;    // Dummy return true for success
}

float DOCKING_STATION_CLASS::getTemperature()
{
    // Do some GPIO stuff here to get the temperature
    return 17.5;    // Dummy temperature
}

float DOCKING_STATION_CLASS::getHumidity()
{
    // Do some GPIO stuff here to get the humidity
    return 5.2;    // Dummy humidity
}

void DOCKING_STATION_CLASS::publicData()
{
    gcs::dockData data;
    data.temperature = getTemperature();
    data.humidity = getHumidity();
    pub.publish(data);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, VALUE_AS_STRING(DOCKING_STATION_CLASS));
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    DOCKING_STATION_CLASS ds(&nh);

    while (ros::ok())
    {
        ds.publicData();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
