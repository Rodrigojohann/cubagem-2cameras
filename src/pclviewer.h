#pragma once
#include <QMainWindow>
#include <QFileDialog>
#include <QColor>
#include <QTimer>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <sensor.h>
#include <controller.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <thread>
#include <math.h>

using namespace std;
typedef std::vector<PointXYZ> CloudVector;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

  public:

  explicit
    PCLViewer (QWidget *parent = 0);

    ~PCLViewer ();

  public Q_SLOTS:

    void FrameBox();
    void FrameGeneric();
    void FrameBoxInPallet();
    void FrameGenericInPallet();
    void CleanBox();
    void CleanGeneric();
    void CleanBoxInPallet();
    void CleanGenericInPallet();

  protected:

    boost::shared_ptr<boost::thread> visualizer_thread_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     inputcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     cloudnew;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coloredinput;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     filteredcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     segmented_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coloredcloud;
    PointCloudT::Ptr                        projectedcloud;
    PointCloudT::Ptr                        cloud_hull;
    std::vector<pcl::Vertices>              hullpolygons;
    std::vector<pcl::PointIndices>          unsortedclusters;
    std::vector<pcl::PointIndices>          notorientedclusters;
    std::vector<pcl::PointIndices>          clusters;
    int                                     cloudcolor[5][3] = {{0, 0, 255}, {0, 220, 0}, {255, 0, 0}, {200, 200, 0}, {255, 0, 255}};
    int                                     clustersize;
    double                                  dimensionX, dimensionY, dimensionZ;
    double                                  minZ;
    double                                  objvolume, totalvolume;
    double                                  volumemean;
    string                                  TotalStr;
    int                                     limitcluster;
    double                                  hullarea;
    double                                  palletarea;
    double                                  minpalletarea;
    int                                     stringprecision;
    double                                  cubefactor;

  private:

    /** @brief ui pointer */
    Ui::PCLViewer *ui;

  public:
    bool firstCall;
};
