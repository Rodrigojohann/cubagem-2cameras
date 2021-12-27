#include "PointXYZ.h"
#include "VisionaryTData.h"
#include "VisionaryDataStream.h"
#include "VisionaryControl.h"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
typedef std::vector<PointXYZ>          CloudVector;

class Sensor
{
public:
    PointCloudT::Ptr CamStream(char* ipAddress, unsigned short port);
    bool TestConnection(char* ipAddress, unsigned short port);
};
