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

    connect (ui->pushButton_2, SIGNAL(pressed()), this, SLOT(CleanGeneric()));
    connect (ui->pushButton_2, SIGNAL(released()), this, SLOT(FrameGeneric()));

    connect (ui->pushButton_3, SIGNAL(pressed()), this, SLOT(CleanBoxInPallet()));
    connect (ui->pushButton_3, SIGNAL(released()), this, SLOT(FrameBoxInPallet()));

    connect (ui->pushButton_4, SIGNAL(pressed()), this, SLOT(CleanGenericInPallet()));
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
    Sensor     s;

    if (s.TestConnection(IP1, PORT) == false or s.TestConnection(IP2, PORT) == false)
    {
        TotalStr = "N/A";
        ui->label_2->setText(QString::fromStdString(TotalStr));
        ui->qvtkWidget->update();
    }
    else
    {
        for (size_t counter = 0; counter < Nsamples; ++counter)
        {
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

                viewer_->updatePointCloud(coloredcloud, to_string(0));
                viewer_->updatePointCloud(coloredcloud, to_string(1));
                viewer_->updatePointCloud(coloredcloud, to_string(2));
                viewer_->updatePointCloud(coloredcloud, to_string(3));
                viewer_->updatePointCloud(coloredcloud, to_string(4));

                for (int number=0; number<limitcluster; ++number)
                {
                    segmented_cloud.reset(new PointCloudT);
                    coloredcloud.reset(new ColoredCloudT);
                    coloredcloud->points.resize(clusters[number].indices.size());
                    segmented_cloud->points.resize(clusters[number].indices.size());

                    for(size_t i=0; i<clusters[number].indices.size(); ++i)
                    {
                        segmented_cloud->points[i].x = (*filteredcloud)[clusters[number].indices[i]].x;
                        segmented_cloud->points[i].y = (*filteredcloud)[clusters[number].indices[i]].y;
                        segmented_cloud->points[i].z = (*filteredcloud)[clusters[number].indices[i]].z;
                    }


                    cloud_planebox = c.ExtractPlaneBox(segmented_cloud);

                    for(size_t i=0; i < cloud_planebox->points.size(); ++i)
                    {
//////////////////

                        coloredcloud->points[i].x = (*cloud_planebox)[i].x;
                        coloredcloud->points[i].y = (*cloud_planebox)[i].y;
                        coloredcloud->points[i].z = (*cloud_planebox)[i].z;

                        coloredcloud->points[i].r = cloudcolor[number][0];
                        coloredcloud->points[i].g = cloudcolor[number][1];
                        coloredcloud->points[i].b = cloudcolor[number][2];
                        coloredcloud->points[i].a = 255;
                    }

                    hullarea = c.SurfaceArea(cloud_planebox);
                    std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensions(cloud_planebox);

                    objvolume = dimensionX*dimensionY*dimensionZ;
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
void PCLViewer::FrameGeneric(){
////////
  Controller c;
  Sensor     s;

  if (s.TestConnection(IP1, PORT) == false or s.TestConnection(IP2, PORT) == false)
  {
      TotalStr = "N/A";
      ui->label_2->setText(QString::fromStdString(TotalStr));
      ui->qvtkWidget->update();
  }
  else
  {
      for (size_t counter = 0; counter < Nsamples; ++counter)
      {
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

              viewer_->updatePointCloud(coloredcloud, to_string(0));
              viewer_->updatePointCloud(coloredcloud, to_string(1));
              viewer_->updatePointCloud(coloredcloud, to_string(2));
              viewer_->updatePointCloud(coloredcloud, to_string(3));
              viewer_->updatePointCloud(coloredcloud, to_string(4));

              for (int number=0; number<limitcluster; ++number)
              {
                  segmented_cloud.reset(new PointCloudT);
                  coloredcloud.reset(new ColoredCloudT);
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
                  std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensionsGeneric(segmented_cloud);

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
void PCLViewer::FrameBoxInPallet(){
  ////////
    Controller c;
    Sensor     s;

    if (s.TestConnection(IP1, PORT) == false or s.TestConnection(IP2, PORT) == false)
    {
        TotalStr = "N/A";
        ui->label_2->setText(QString::fromStdString(TotalStr));
        ui->qvtkWidget->update();
    }
    else
    {
        for (size_t counter = 0; counter < Nsamples; ++counter)
        {
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
                cloud_palletremoved = c.RemovePallet(filteredcloud);
                std::tie(unsortedclusters, clustersize) = c.CloudSegmentation(cloud_palletremoved);

                notorientedclusters = c.SortClusters(unsortedclusters, clustersize);
                clusters = c.RemoveInclined(cloud_palletremoved, notorientedclusters);

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

                viewer_->updatePointCloud(coloredcloud, to_string(0));
                viewer_->updatePointCloud(coloredcloud, to_string(1));
                viewer_->updatePointCloud(coloredcloud, to_string(2));
                viewer_->updatePointCloud(coloredcloud, to_string(3));
                viewer_->updatePointCloud(coloredcloud, to_string(4));

                for (int number=0; number<limitcluster; ++number)
                {
                    segmented_cloud.reset(new PointCloudT);
                    coloredcloud.reset(new ColoredCloudT);
                    coloredcloud->points.resize(clusters[number].indices.size());
                    segmented_cloud->points.resize(clusters[number].indices.size());

                    for(size_t i=0; i<clusters[number].indices.size(); ++i)
                    {
                        segmented_cloud->points[i].x = (*cloud_palletremoved)[clusters[number].indices[i]].x;
                        segmented_cloud->points[i].y = (*cloud_palletremoved)[clusters[number].indices[i]].y;
                        segmented_cloud->points[i].z = (*cloud_palletremoved)[clusters[number].indices[i]].z;

                        coloredcloud->points[i].x = (*cloud_palletremoved)[clusters[number].indices[i]].x;
                        coloredcloud->points[i].y = (*cloud_palletremoved)[clusters[number].indices[i]].y;
                        coloredcloud->points[i].z = (*cloud_palletremoved)[clusters[number].indices[i]].z;

                        coloredcloud->points[i].r = cloudcolor[number][0];
                        coloredcloud->points[i].g = cloudcolor[number][1];
                        coloredcloud->points[i].b = cloudcolor[number][2];
                        coloredcloud->points[i].a = 255;
                    }

                    hullarea = c.SurfaceArea(segmented_cloud);
                    std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensions(segmented_cloud);

                    dimensionZ = dimensionZ - (PALLETHEIGHT*100);

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
void PCLViewer::FrameGenericInPallet(){
////////
Controller c;
Sensor     s;

if (s.TestConnection(IP1, PORT) == false or s.TestConnection(IP2, PORT) == false)
{
  TotalStr = "N/A";
  ui->label_2->setText(QString::fromStdString(TotalStr));
  ui->qvtkWidget->update();
}
else
{
  for (size_t counter = 0; counter < Nsamples; ++counter)
  {
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
          cloud_palletremoved = c.RemovePallet(filteredcloud);
          std::tie(unsortedclusters, clustersize) = c.CloudSegmentation(cloud_palletremoved);

          notorientedclusters = c.SortClusters(unsortedclusters, clustersize);
          clusters = c.RemoveInclined(cloud_palletremoved, notorientedclusters);

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

          viewer_->updatePointCloud(coloredcloud, to_string(0));
          viewer_->updatePointCloud(coloredcloud, to_string(1));
          viewer_->updatePointCloud(coloredcloud, to_string(2));
          viewer_->updatePointCloud(coloredcloud, to_string(3));
          viewer_->updatePointCloud(coloredcloud, to_string(4));

          for (int number=0; number<limitcluster; ++number)
          {
              segmented_cloud.reset(new PointCloudT);
              coloredcloud.reset(new ColoredCloudT);
              coloredcloud->points.resize(clusters[number].indices.size());
              segmented_cloud->points.resize(clusters[number].indices.size());

              for(size_t i=0; i<clusters[number].indices.size(); ++i)
              {
                  segmented_cloud->points[i].x = (*cloud_palletremoved)[clusters[number].indices[i]].x;
                  segmented_cloud->points[i].y = (*cloud_palletremoved)[clusters[number].indices[i]].y;
                  segmented_cloud->points[i].z = (*cloud_palletremoved)[clusters[number].indices[i]].z;

                  coloredcloud->points[i].x = (*cloud_palletremoved)[clusters[number].indices[i]].x;
                  coloredcloud->points[i].y = (*cloud_palletremoved)[clusters[number].indices[i]].y;
                  coloredcloud->points[i].z = (*cloud_palletremoved)[clusters[number].indices[i]].z;

                  coloredcloud->points[i].r = cloudcolor[number][0];
                  coloredcloud->points[i].g = cloudcolor[number][1];
                  coloredcloud->points[i].b = cloudcolor[number][2];
                  coloredcloud->points[i].a = 255;
              }

              hullarea = c.SurfaceArea(segmented_cloud);
              std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensionsGeneric(segmented_cloud);

              dimensionZ = dimensionZ - (PALLETHEIGHT*100);
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
void PCLViewer::CleanBox()
{
    viewer_->removeAllPointClouds();

    coloredinput.reset(new ColoredCloudT);
    coloredcloud.reset(new ColoredCloudT);

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
void PCLViewer::CleanGeneric()
{
    viewer_->removeAllPointClouds();

    coloredinput.reset(new ColoredCloudT);
    coloredcloud.reset(new ColoredCloudT);

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
void PCLViewer::CleanBoxInPallet()
{
    viewer_->removeAllPointClouds();

    coloredinput.reset(new ColoredCloudT);
    coloredcloud.reset(new ColoredCloudT);

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
void PCLViewer::CleanGenericInPallet()
{
    viewer_->removeAllPointClouds();

    coloredinput.reset(new ColoredCloudT);
    coloredcloud.reset(new ColoredCloudT);

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
PCLViewer::~PCLViewer()
{
  delete ui;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
