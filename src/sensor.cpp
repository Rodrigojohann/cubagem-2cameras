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
PointCloudT::Ptr Sensor::TwoCamStream(char* ipAddress1, char* ipAddress2, unsigned short port)
{
// var
    PointCloudT::Ptr                  cloud_raw1 (new PointCloudT);
    PointCloudT::Ptr                  cloud_raw2 (new PointCloudT);
    PointCloudT::Ptr                  cloud_transformed (new PointCloudT);
    PointCloudT::Ptr                  cloud_aligned (new PointCloudT);
    PointCloudT::Ptr                  output_cloud (new PointCloudT);
    PointCloudT::Ptr                  cloud_transformed1 (new PointCloudT);
    PointCloudT::Ptr                  cloud_transformed2 (new PointCloudT);
    PointCloudT::Ptr                  cloud_transformed3 (new PointCloudT);
    pcl::PassThrough<pcl::PointXYZ>   passz;
////
    cloud_raw1.reset(new PointCloudT);
    cloud_raw2.reset(new PointCloudT);

    cloud_raw1 = CamStream(ipAddress1, port);
    cloud_raw2 = CamStream(ipAddress2, port);

cout << "\n\nCloud 1: " << cloud_raw1->points.size();
cout << "\nCloud 2: " << cloud_raw2->points.size();

passz.setInputCloud(cloud_raw1);
passz.setFilterFieldName ("z");
passz.setFilterLimits (0, 5);
passz.filter(*cloud_raw1);

passz.setInputCloud(cloud_raw2);
passz.setFilterFieldName ("z");
passz.setFilterLimits (0, 5);
passz.filter(*cloud_raw2);

cout << "\n\nCloud 1 filtered: " << cloud_raw1->points.size();
cout << "\nCloud 2 filtered: " << cloud_raw2->points.size();

float theta1 = -ANGLE1*(M_PI/180);
float theta2 = -ANGLE2*(M_PI/180);

Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

transform_1.rotate (Eigen::AngleAxisf (theta1, Eigen::Vector3f::UnitX()));
transform_2.rotate (Eigen::AngleAxisf (theta2, Eigen::Vector3f::UnitX()));

std::cout << transform_2.matrix() << std::endl;

pcl::transformPointCloud (*cloud_raw1, *cloud_transformed1, transform_1);
pcl::transformPointCloud (*cloud_raw2, *cloud_transformed2, transform_2);


float theta3 = M_PI;
Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
transform_3.rotate (Eigen::AngleAxisf (theta3, Eigen::Vector3f::UnitZ()));
transform_3.translation() << 0.0, 3.05, 0.0;

pcl::transformPointCloud (*cloud_transformed2, *cloud_transformed3, transform_3);


*output_cloud += *cloud_transformed1;
*output_cloud += *cloud_transformed3;

pcl::VoxelGrid<pcl::PointXYZ> sor;
sor.setInputCloud (output_cloud);
sor.setLeafSize (0.01f, 0.01f, 0.01f);
sor.filter (*output_cloud);

    return output_cloud;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr Sensor::RemoveDistortion(PointCloudT::Ptr inputcloud)
{
// var
    PointCloudT::Ptr outputcloud(new PointCloudT);
    double xi, yi, zi, xi_0, yi_0, zi_0;
    double Ri;
    double M;
////

    outputcloud->points.resize(inputcloud->points.size());

    for(size_t i=0; i<outputcloud->points.size(); ++i)
    {
        xi_0 = (*inputcloud)[i].x;
        yi_0 = (*inputcloud)[i].y;
        zi_0 = (*inputcloud)[i].z;

        Ri = sqrt(pow(xi_0, 2.0)+pow(yi_0, 2.0)+pow(zi_0, 2.0));
        M = 1-0.5*((pow(xi_0, 2.0)*pow(tan(XANGLE), 2.0))+(pow(yi_0, 2.0)*pow(tan(YANGLE), 2.0)));

        xi = xi_0*M*tan(XANGLE)*Ri;
        yi = yi_0*M*tan(YANGLE)*Ri;
        zi = M*Ri;

        outputcloud->points[i].x = (xi);
        outputcloud->points[i].y = (yi);
        outputcloud->points[i].z = (zi);
    }

    return outputcloud;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
