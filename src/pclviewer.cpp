#include "pclviewer.h"
#include "ui_pclviewer.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
////
{
    ui->setupUi (this);
    //
//    this->setWindowState(Qt::WindowFullScreen);
//    this->setCursor(Qt::BlankCursor);
    this->setWindowTitle ("Cubagem");

    connect (ui->pushButton_1, SIGNAL(pressed()), this, SLOT(CleanBox()));
    connect (ui->pushButton_1, SIGNAL(released()), this, SLOT(FrameBox()));

    connect (ui->pushButton_2, SIGNAL(pressed()), this, SLOT(CleanPallet()));
    connect (ui->pushButton_2, SIGNAL(released()), this, SLOT(FrameGeneric()));

    connect (ui->pushButton_3, SIGNAL(pressed()), this, SLOT(CleanBoxInPallet()));
    connect (ui->pushButton_3, SIGNAL(released()), this, SLOT(FrameBoxInPallet()));

    connect (ui->pushButton_4, SIGNAL(pressed()), this, SLOT(CleanGeneric()));
    connect (ui->pushButton_4, SIGNAL(released()), this, SLOT(FrameGenericInPallet()));

    // Set up the QVTK window
    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->setShowFPS(false);
    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
//    ui->qvtkWidget->setEnabled(false);

    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void PCLViewer::FrameBox(){
  ////////
    Controller c;
    Sensor s;

    if (s.TestConnection(IP1, PORT) == false and s.TestConnection(IP2, PORT) == false)
    {
        TotalStr = "N/A";
        ui->label_2->setText(QString::fromStdString(TotalStr));
        ui->qvtkWidget->update();
    }
    else
    {
        for (size_t counter = 0; counter < Nsamples; ++counter)
        {
            cloudnew.reset(new pcl::PointCloud<pcl::PointXYZ>);
            coloredinput.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

            //cloudnew = s.CamStream(IP, PORT);
            cloudnew = s.TwoCamStream(IP1, IP2, PORT);
            coloredinput->points.resize(cloudnew->points.size());

            if (cloudnew->points.size() > 100){
                for (size_t i = 0; i < coloredinput->points.size(); i++)
                {
                    coloredinput->points[i].x = (*cloudnew)[i].x;
                    coloredinput->points[i].y = (*cloudnew)[i].y;
                    coloredinput->points[i].z = (*cloudnew)[i].z;
                    coloredinput->points[i].r = 255;
                    coloredinput->points[i].g = 255;
                    coloredinput->points[i].b = 255;
                    coloredinput->points[i].a = 200;
                }

                filteredcloud = c.FilterCloud(cloudnew);
                std::tie(unsortedclusters, clustersize) = c.CloudSegmentation(filteredcloud);

                notorientedclusters = c.SortClusters(unsortedclusters, clustersize);
                clusters = c.RemoveInclined(filteredcloud, notorientedclusters);

                viewer_->updatePointCloud(coloredinput, "inputcloud");

                if (clusters.size() > 5)
                {
                    limitcluster = 5;
                }
                else
                {
                    limitcluster = clusters.size();
                }

                totalvolume = 0.0;
                objvolume = 0.0;

                coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

                viewer_->updatePointCloud(coloredcloud, to_string(0));
                viewer_->updatePointCloud(coloredcloud, to_string(1));
                viewer_->updatePointCloud(coloredcloud, to_string(2));
                viewer_->updatePointCloud(coloredcloud, to_string(3));
                viewer_->updatePointCloud(coloredcloud, to_string(4));

                for (int number=0; number<limitcluster; ++number)
                {
                    segmented_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
                    coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    coloredcloud->points.resize(clusters[number].indices.size());
                    segmented_cloud->points.resize(clusters[number].indices.size());

                    for(size_t i=0; i<clusters[number].indices.size(); ++i)
                    {
                        segmented_cloud->points[i].x = (*filteredcloud)[clusters[number].indices[i]].x;
                        segmented_cloud->points[i].y = (*filteredcloud)[clusters[number].indices[i]].y;
                        segmented_cloud->points[i].z = (*filteredcloud)[clusters[number].indices[i]].z;

                        coloredcloud->points[i].x = (*filteredcloud)[clusters[number].indices[i]].x;
                        coloredcloud->points[i].y = (*filteredcloud)[clusters[number].indices[i]].y;
                        coloredcloud->points[i].z = (*filteredcloud)[clusters[number].indices[i]].z;

                        coloredcloud->points[i].r = cloudcolor[number][0];
                        coloredcloud->points[i].g = cloudcolor[number][1];
                        coloredcloud->points[i].b = cloudcolor[number][2];
                        coloredcloud->points[i].a = 255;
                    }

                    hullarea = c.SurfaceArea(segmented_cloud);
                    std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensions(segmented_cloud);

                    objvolume = hullarea*dimensionZ;
                    totalvolume += objvolume;

                    viewer_->updatePointCloud(coloredcloud, to_string(number));
                }
            }
            volumemean += totalvolume;
        }

        cubefactor = (volumemean/Nsamples)/5988.02395;
        cubefactor = floor((cubefactor*2)+0.5)/2;

        if (cubefactor >= 1000)
        {
            stringprecision = 6;
        }
        else if (cubefactor < 1000 and cubefactor >= 100)
        {
            stringprecision = 5;
        }
        else if (cubefactor < 100 and cubefactor >= 10)
        {
            stringprecision = 4;
        }
        else
        {
            stringprecision = 3;
        }

        TotalStr = to_string(cubefactor).substr(0,stringprecision)+" kg";
        ui->label_2->setText(QString::fromStdString(TotalStr));
        ui->qvtkWidget->update();
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::FrameGeneric()
{
    ////////
    Controller c;
    Sensor s;

    if (s.TestConnection(IP1, PORT) == false and s.TestConnection(IP2, PORT) == false)
    {
        TotalStr = "N/A";
        ui->label_2->setText(QString::fromStdString(TotalStr));
        ui->qvtkWidget->update();
    }
    else
    {
        for (size_t counter = 0; counter < Nsamples; ++counter)
        {
            cloudnew.reset(new pcl::PointCloud<pcl::PointXYZ>);
            coloredinput.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

            //cloudnew = s.CamStream(IP, PORT);
            cloudnew = s.TwoCamStream(IP1, IP2, PORT);
            coloredinput->points.resize(cloudnew->points.size());

            if (cloudnew->points.size() > 100)
            {
                for (size_t i = 0; i < coloredinput->points.size(); i++)
                {
                    coloredinput->points[i].x = (*cloudnew)[i].x;
                    coloredinput->points[i].y = (*cloudnew)[i].y;
                    coloredinput->points[i].z = (*cloudnew)[i].z;
                    coloredinput->points[i].r = 255;
                    coloredinput->points[i].g = 255;
                    coloredinput->points[i].b = 255;
                    coloredinput->points[i].a = 200;
                }

                filteredcloud = c.FilterCloud(cloudnew);

                viewer_->updatePointCloud(coloredinput, "inputcloud");

                coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
                viewer_->updatePointCloud (coloredcloud, to_string(0));

                coloredcloud->points.resize(filteredcloud->points.size());

                for(size_t i=0; i<filteredcloud->points.size(); ++i)
                {
                    coloredcloud->points[i].x = (*filteredcloud)[i].x;
                    coloredcloud->points[i].y = (*filteredcloud)[i].y;
                    coloredcloud->points[i].z = (*filteredcloud)[i].z;

                    coloredcloud->points[i].r = cloudcolor[0][0];
                    coloredcloud->points[i].g = cloudcolor[0][1];
                    coloredcloud->points[i].b = cloudcolor[0][2];
                    coloredcloud->points[i].a = 255;
                }


                cloud_hull.reset(new pcl::PointCloud<pcl::PointXYZ>);
                std::tie(cloud_hull, hullpolygons) = c.ConvexHull(filteredcloud);

                //viewer_->addPolygonMesh<pcl::PointXYZ>(cloud_hull, hullpolygons, "polyline");

                projectedcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
                projectedcloud = c.ProjectCloud(filteredcloud);

                std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensionsPallet(filteredcloud);
                palletarea = c.PalletArea(projectedcloud);

                if (counter == 0)
                {
                    minpalletarea = palletarea;
                    minZ = dimensionZ;
                }
                else
                {
                    if (palletarea < minpalletarea)
                    {
                        minpalletarea = palletarea;
                    }

                    if (dimensionZ < minZ)
                    {
                        minZ = dimensionZ;
                    }
                }
                viewer_->updatePointCloud (coloredcloud, to_string(0));
            }
        }

        cubefactor = (minpalletarea*minZ)/5988.02395;
        cubefactor = floor((cubefactor*2)+0.5)/2;

        if (cubefactor >= 1000)
        {
            stringprecision = 6;
        }
        else if (cubefactor < 1000 and cubefactor >= 100)
        {
            stringprecision = 5;
        }
        else if (cubefactor < 100 and cubefactor >= 10)
        {
            stringprecision = 4;
        }
        else
        {
            stringprecision = 3;
        }

        TotalStr = to_string(cubefactor).substr(0,stringprecision)+" kg";
        ui->label_2->setText(QString::fromStdString(TotalStr));
        ui->qvtkWidget->update();
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::FrameBoxInPallet()
{

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::FrameGenericInPallet()
{

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::CleanBox()
{
    viewer_->removeAllPointClouds();

    coloredinput.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    viewer_->resetCamera();
    viewer_->setCameraPosition(0, 0, -3, 0, -1.3, -1);

    viewer_->addPointCloud(coloredinput, "inputcloud");
    viewer_->addPointCloud(coloredcloud, to_string(0));
    viewer_->addPointCloud(coloredcloud, to_string(1));
    viewer_->addPointCloud(coloredcloud, to_string(2));
    viewer_->addPointCloud(coloredcloud, to_string(3));
    viewer_->addPointCloud(coloredcloud, to_string(4));

    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "inputcloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(0));
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(1));
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(2));
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(3));
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(4));

    volumemean = 0.0;

    ui->label_2->setStyleSheet("font-weight: bold");
    ui->label_2->setText(QString::fromStdString("0.0 kg"));
    ui->qvtkWidget->update();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::CleanPallet()
{
    viewer_->removeAllPointClouds();

    coloredinput.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    viewer_->resetCamera();
    viewer_->setCameraPosition(0, 0, -3, 0, -1.3, -1);

    viewer_->addPointCloud(coloredinput, "inputcloud");
    viewer_->addPointCloud(coloredcloud, to_string(0));

    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "inputcloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(0));

    minZ = 0.0;

    ui->label_2->setStyleSheet("font-weight: bold");
    ui->label_2->setText(QString::fromStdString("0.0 kg"));
    ui->qvtkWidget->update();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::CleanBoxInPallet()
{

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::CleanGeneric()
{

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PCLViewer::~PCLViewer()
{
  delete ui;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
