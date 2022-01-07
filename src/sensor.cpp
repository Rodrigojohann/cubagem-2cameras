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
    cloud_raw1.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_raw2.reset(new pcl::PointCloud<pcl::PointXYZ>);

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

//cloud_transformed1 = RemoveDistortion(cloud_raw1);
//cloud_transformed2 = RemoveDistortion(cloud_transformed3);

//pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;



//pcl::PointXYZ min_point_OBB1;
//pcl::PointXYZ max_point_OBB1;
//pcl::PointXYZ position_OBB1;
//Eigen::Matrix3f rotational_matrix_OBB1;

//feature_extractor.setInputCloud (cloud_transformed);
//feature_extractor.compute ();
//feature_extractor.getOBB (min_point_OBB1, max_point_OBB1, position_OBB1, rotational_matrix_OBB1);

//Eigen::Matrix3f rotated_matrix1 = rotational_matrix_OBB1.inverse();

//pcl::PointXYZ min_point_OBB2;
//pcl::PointXYZ max_point_OBB2;
//pcl::PointXYZ position_OBB2;
//Eigen::Matrix3f rotational_matrix_OBB2;

//feature_extractor.setInputCloud (cloud_raw2);
//feature_extractor.compute ();
//feature_extractor.getOBB (min_point_OBB2, max_point_OBB2, position_OBB2, rotational_matrix_OBB2);

//Eigen::Matrix3f rotated_matrix2 = rotational_matrix_OBB2.inverse();

//Eigen::Matrix4f transform_align1 = Eigen::Matrix4f::Identity();

//transform_align1 (0,0) = rotated_matrix1 (0,0);
//transform_align1 (0,1) = rotated_matrix1 (0,1);
//transform_align1 (0,2) = rotated_matrix1 (0,2);
//transform_align1 (1,0) = rotated_matrix1 (1,0);
//transform_align1 (1,1) = rotated_matrix1 (1,1);
//transform_align1 (1,2) = rotated_matrix1 (1,2);
//transform_align1 (2,0) = rotated_matrix1 (2,0);
//transform_align1 (2,1) = rotated_matrix1 (2,1);
//transform_align1 (2,2) = rotated_matrix1 (2,2);

//transform_align1 (0,3) = -position_OBB1.x;./cu

//transform_align1 (1,3) = -position_OBB1.y;
//transform_align1 (2,3) = -position_OBB1.z;

//Eigen::Matrix4f transform_align2 = Eigen::Matrix4f::Identity();

//transform_align2 (0,0) = rotated_matrix2 (0,0);
//transform_align2 (0,1) = rotated_matrix2 (0,1);
//transform_align2 (0,2) = rotated_matrix2 (0,2);
//transform_align2 (1,0) = rotated_matrix2 (1,0);
//transform_align2 (1,1) = rotated_matrix2 (1,1);
//transform_align2 (1,2) = rotated_matrix2 (1,2);
//transform_align2 (2,0) = rotated_matrix2 (2,0);
//transform_align2 (2,1) = rotated_matrix2 (2,1);
//transform_align2 (2,2) = rotated_matrix2 (2,2);

//transform_align2 (0,3) = -position_OBB1.x;
//transform_align2 (1,3) = -position_OBB1.y;
//transform_align2 (2,3) = -position_OBB1.z;


//pcl::transformPointCloud (*cloud_transformed, *cloud_transformed1, rotational_matrix_OBB1.inverse());
//pcl::transformPointCloud (*cloud_raw2, *cloud_transformed2, rotational_matrix_OBB2.inverse());



//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//icp.setInputSource(cloud_transformed);
//icp.setInputTarget(cloud_raw2);

//icp.align(*cloud_aligned);

//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//icp.getFitnessScore() << std::endl;
//std::cout << icp.getFinalTransformation() << std::endl;

*output_cloud += *cloud_transformed1;
//*output_cloud += *cloud_raw2;
*output_cloud += *cloud_transformed3;

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
