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
typedef std::vector<PointXYZ>              CloudVector;
typedef pcl::PointCloud<pcl::PointXYZ>     PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> ColoredCloudT;

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

    boost::shared_ptr<boost::thread>                     visualizer_thread_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    PointCloudT::Ptr                                     cloudnew;
    PointCloudT::Ptr                                     cloud_preprocessed;
    PointCloudT::Ptr                                     filteredcloud;
    PointCloudT::Ptr                                     cloud_palletremoved;
    PointCloudT::Ptr                                     cloud_hull;
    ColoredCloudT::Ptr                                   coloredinput;
    ColoredCloudT::Ptr                                   coloredcloud;
    std::vector<pcl::Vertices>                           hullpolygons;
    std::vector<pcl::PointIndices>                       notorientedclusters;
    std::vector<PointCloudT::Ptr>                        clusters;
    int                                                  cloudcolor[10][3] = {{0, 0, 255}, {0, 220, 0}, {255, 0, 0}, {200, 200, 0}, {255, 0, 255}, {0, 255, 150}, {125, 0, 125}, {0, 255, 255}, {170, 0, 255}, {0, 150, 150}};
    int                                                  clustersize;
    int                                                  limitcluster;
    int                                                  stringprecision;
    double                                               dimensionX, dimensionY, dimensionZ;
    double                                               minZ;
    double                                               objvolume, totalvolume;
    double                                               volumemean;
    double                                               hullarea;
    double                                               palletarea;
    double                                               minpalletarea;
    double                                               cubefactor;
    string                                               TotalStr;

  private:

    /** @brief ui pointer */
    Ui::PCLViewer *ui;

  public:
    bool firstCall;
};
