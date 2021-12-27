#include "sensor.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr Sensor::CamStream(char* ipAddress, unsigned short port)
{
// var
    CloudVector       			      pointCloud;
    boost::shared_ptr<VisionaryTData> pDataHandler;
    PointCloudT::Ptr cloud_raw (new PointCloudT);
////
    // Generate Visionary instance
    pDataHandler = boost::make_shared<VisionaryTData>();
    VisionaryDataStream dataStream(pDataHandler, inet_addr(ipAddress), htons(port));
    VisionaryControl control(inet_addr(ipAddress), htons(2112));

    // Connect to devices data stream
    if (!dataStream.openConnection())
    {
        printf("Failed to open data stream connection to device.\n");
        return cloud_raw;
    }
    //-----------------------------------------------
    // Connect to devices control channel
    if (!control.openConnection())
    {
        printf("Failed to open control connection to device.\n");
        return cloud_raw;
    }
    control.stopAcquisition();
    control.startAcquisition();

    if (dataStream.getNextFrame())
    {
        // Convert data to a point cloud
        pDataHandler->generatePointCloud(pointCloud);
    }

    cloud_raw->points.resize(pointCloud.size());
    for(size_t i=0; i<cloud_raw->points.size(); ++i)
    {
        cloud_raw->points[i].x = pointCloud[i].x;
        cloud_raw->points[i].y = pointCloud[i].y;
        cloud_raw->points[i].z = pointCloud[i].z;
    }

    control.stopAcquisition();
    control.closeConnection();
    dataStream.closeConnection();

    return cloud_raw;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Sensor::TestConnection(char* ipAddress, unsigned short port)
{
// var
    boost::shared_ptr<VisionaryTData> pDataHandler;
    VisionaryDataStream dataStream(pDataHandler, inet_addr(ipAddress), htons(port));
    VisionaryControl control(inet_addr(ipAddress), htons(2112));
////
    if (dataStream.openConnection() && control.openConnection())
    {
        control.closeConnection();
        dataStream.closeConnection();
        return true;
    }
    else
    {
        return false;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
