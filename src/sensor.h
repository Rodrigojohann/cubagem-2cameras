#include "PointXYZ.h"
#include "VisionaryTData.h"
#include "VisionaryDataStream.h"
#include "VisionaryControl.h"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <config.h>

#include <pcl/filters/voxel_grid.h>

#define XANGLE 0.60213859194
#define YANGLE 0.48869219056

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
typedef std::vector<PointXYZ>          CloudVector;

class Sensor
{
public:
    PointCloudT::Ptr CamStream(char* ipAddress, unsigned short port);
    bool TestConnection(char* ipAddress, unsigned short port);
    PointCloudT::Ptr TwoCamStream(char* ipAddress1, char* ipAddress2, unsigned short port);
    PointCloudT::Ptr RemoveDistortion(PointCloudT::Ptr inputcloud);
};
