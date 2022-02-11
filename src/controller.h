#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <config.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

class Controller
{
public:
    PointCloudT::Ptr                                PreProcessingCloud(PointCloudT::Ptr inputcloud);
    PointCloudT::Ptr                                FilterROI(PointCloudT::Ptr inputcloud);
    PointCloudT::Ptr                                RemovePallet(PointCloudT::Ptr inputcloud);
    std::vector<pcl::PointIndices>                  CloudSegmentation(PointCloudT::Ptr inputcloud);
    bool                                            ClusterCondition(PointCloudT::Ptr seedPoint, PointCloudT::Ptr candidatePoint);
    std::vector<pcl::PointIndices>                  CloudSegmentationPallet(PointCloudT::Ptr inputcloud);
    std::tuple<float, float, float>                 CalculateDimensions(PointCloudT::Ptr inputcloud);
    std::tuple<float, float, float>                 CalculateDimensionsGeneric(PointCloudT::Ptr inputcloud);
    std::vector <PointCloudT::Ptr>                  ExtractTopPlaneBox(PointCloudT::Ptr inputcloud, std::vector<pcl::PointIndices> inputclusters);
    std::vector <PointCloudT::Ptr>                  IndicestoClouds(PointCloudT::Ptr inputcloud, std::vector<pcl::PointIndices> inputindices);
    double                                          SurfaceArea(PointCloudT::Ptr inputcloud);
    double                                          PalletArea(PointCloudT::Ptr inputcloud);
    PointCloudT::Ptr                                ProjectCloud(PointCloudT::Ptr inputcloud);
    std::tuple<PointCloudT::Ptr, std::vector<pcl::Vertices>> ConvexHull(PointCloudT::Ptr inputcloud);    
};
