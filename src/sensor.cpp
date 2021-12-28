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
    PointCloudT::Ptr                  cloud_sum (new PointCloudT);
    pcl::PassThrough<pcl::PointXYZ>                passz;
////
    cloud_raw1.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_raw2.reset(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_raw1 = CamStream(ipAddress1, port);
    cloud_raw2 = CamStream(ipAddress2, port);

cout << "\n\nCloud 1: " << cloud_raw1->points.size();
cout << "\nCloud 2: " << cloud_raw2->points.size();

passz.setInputCloud(cloud_raw1);
passz.setFilterFieldName ("z");
passz.setFilterLimits (-5000, 5000);
passz.filter(*cloud_raw1);

passz.setInputCloud(cloud_raw2);
passz.setFilterFieldName ("z");
passz.setFilterLimits (-5000, 5000);
passz.filter(*cloud_raw2);

pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
approximate_voxel_filter.setInputCloud (cloud_raw1);
approximate_voxel_filter.filter (*filtered_cloud1);

cout << "\n\nCloud 1 filtered: " << cloud_raw1->points.size();
cout << "\nCloud 2 filtered: " << cloud_raw2->points.size();

pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

ndt.setTransformationEpsilon (0.01);
ndt.setStepSize (0.1);
ndt.setResolution (1.0);

ndt.setMaximumIterations (35);

ndt.setInputSource (filtered_cloud1);
ndt.setInputTarget (cloud_raw2);

Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

ndt.align (*cloud_sum, init_guess);

*cloud_sum += *cloud_raw1;

    return cloud_sum+cloud_raw2;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
