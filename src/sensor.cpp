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
    pcl::PassThrough<pcl::PointXYZ>   passz;
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

cout << "\n\nCloud 1 filtered: " << cloud_raw1->points.size();
cout << "\nCloud 2 filtered: " << cloud_raw2->points.size();


float theta = M_PI;
Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

std::cout << transform_2.matrix() << std::endl;

pcl::transformPointCloud (*cloud_raw1, *cloud_transformed, transform_2);


pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;



pcl::PointXYZ min_point_OBB1;
pcl::PointXYZ max_point_OBB1;
pcl::PointXYZ position_OBB1;
Eigen::Matrix3f rotational_matrix_OBB1;

feature_extractor.setInputCloud (cloud_transformed);
feature_extractor.compute ();
feature_extractor.getOBB (min_point_OBB1, max_point_OBB1, position_OBB1, rotational_matrix_OBB1);

pcl::PointXYZ min_point_OBB2;
pcl::PointXYZ max_point_OBB2;
pcl::PointXYZ position_OBB2;
Eigen::Matrix3f rotational_matrix_OBB2;

feature_extractor.setInputCloud (cloud_raw2);
feature_extractor.compute ();
feature_extractor.getOBB (min_point_OBB2, max_point_OBB2, position_OBB2, rotational_matrix_OBB2);

pcl::transformPointCloud (*cloud_transformed, *cloud_transformed1, rotational_matrix_OBB1.inverse());
pcl::transformPointCloud (*cloud_raw2, *cloud_transformed2, rotational_matrix_OBB2.inverse());



//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//icp.setInputSource(cloud_transformed);
//icp.setInputTarget(cloud_raw2);

//icp.align(*cloud_aligned);

//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//icp.getFitnessScore() << std::endl;
//std::cout << icp.getFinalTransformation() << std::endl;

*output_cloud += *cloud_aligned;
*output_cloud += *cloud_raw2;

    return cloud_raw1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
