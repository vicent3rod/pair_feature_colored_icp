//// Many of the code base on: https://github.com/tttamaki/ICP-test/blob/master/src/icp4_after_feature_registration.cpp

#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>
#include <boost/random.hpp>
///
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
///
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
//
#include "visualize_correspondences.h"
//cloud xyz
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
//cloud color
typedef pcl::PointXYZRGB PointTColor;
typedef pcl::PointCloud<PointTColor> PointCloudColor;

void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
              pcl::PointCloud<pcl::Normal>::Ptr normals,
              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
    searchTree->setInputCloud ( cloud );

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud ( cloud );
    normalEstimator.setSearchMethod ( searchTree );
    normalEstimator.setKSearch ( 15 );
    normalEstimator.compute ( *normals );

    pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
}

void loadCloud(const char* fileName, 
            pcl::PointCloud<pcl::PointXYZ> &cloud) 
{
    pcl::PolygonMesh mesh;
    
    if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 ){
        PCL_ERROR ( "loadCloud faild." );
        return;
    } else {
        pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
    }
    
    std::vector<int> index;
    pcl::removeNaNFromPointCloud ( cloud, cloud, index ); // remove points having values of nan

}

void loadCloud_Color(const char* fileName, 
                    pcl::PointCloud<pcl::PointXYZRGB> &cloud) 
{ 
  pcl::PolygonMesh mesh;

  if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 ){
      PCL_ERROR ( "loadCloud faild." );
      return;
  } else {
      pcl::fromPCLPointCloud2<pcl::PointXYZRGB> ( mesh.cloud, cloud );
  }
  
  std::vector<int> index;
  pcl::removeNaNFromPointCloud ( cloud, cloud, index ); // remove points having values of nan

}

// from: https://stackoverflow.com/questions/59395218/pcl-scale-two-point-clouds-to-the-same-size
void rescaleClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& outputCloud, 
                    bool debugOverlay = false, 
                    bool primaryAxisOnly = true){
    
    //analyze golden cloud
    pcl::PCA<pcl::PointXYZ> pcaGolden;
    pcaGolden.setInputCloud(inputCloud);
    Eigen::Matrix3f goldenEVs_Dir = pcaGolden.getEigenVectors();
    Eigen::Vector4f goldenMidPt = pcaGolden.getMean();
    Eigen::Matrix4f goldenTransform = Eigen::Matrix4f::Identity();
    goldenTransform.block<3, 3>(0, 0) = goldenEVs_Dir;
    goldenTransform.block<4, 1>(0, 3) = goldenMidPt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr orientedGolden(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*inputCloud, *orientedGolden, goldenTransform.inverse());
    pcl::PointXYZ goldenMin, goldenMax;
    pcl::getMinMax3D(*orientedGolden, goldenMin, goldenMax);

    //analyze sample cloud
    pcl::PCA<pcl::PointXYZ> pcaSample;
    pcaSample.setInputCloud(outputCloud);
    Eigen::Matrix3f sampleEVs_Dir = pcaSample.getEigenVectors();
    Eigen::Vector4f sampleMidPt = pcaSample.getMean();
    Eigen::Matrix4f sampleTransform = Eigen::Matrix4f::Identity();
    sampleTransform.block<3, 3>(0, 0) = sampleEVs_Dir;
    sampleTransform.block<4, 1>(0, 3) = sampleMidPt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr orientedSample(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*outputCloud, *orientedSample, sampleTransform.inverse());
    pcl::PointXYZ sampleMin, sampleMax;
    pcl::getMinMax3D(*orientedSample, sampleMin, sampleMax);

    //apply scaling to oriented sample cloud 
    double xScale = (sampleMax.x - sampleMin.x) / (goldenMax.x - goldenMin.x);
    double yScale = (sampleMax.y - sampleMin.y) / (goldenMax.y - goldenMin.y);
    double zScale = (sampleMax.z - sampleMin.z) / (goldenMax.z - goldenMin.z);

    if (primaryAxisOnly) { 
      std::cout << "scale: " << xScale << std::endl; }
    else { 
      std::cout << "xScale: " << xScale << " | yScale: " << yScale << " | zScale: " << zScale << std::endl; 
    }

    for (int i = 0; i < orientedSample->points.size(); i++) {
        
        if (primaryAxisOnly){
            orientedSample->points[i].x = orientedSample->points[i].x / xScale;
            orientedSample->points[i].y = orientedSample->points[i].y / xScale;
            orientedSample->points[i].z = orientedSample->points[i].z / xScale;
        }
        else{
            orientedSample->points[i].x = orientedSample->points[i].x / xScale;
            orientedSample->points[i].y = orientedSample->points[i].y / yScale;
            orientedSample->points[i].z = orientedSample->points[i].z / zScale;
        }
    }
    //depending on your next step, it may be reasonable to leave this cloud at its eigen orientation, but this transformation will allow this function to scale in place.

    if (debugOverlay){
        pcl::transformPointCloud(*orientedGolden, *inputCloud, sampleTransform);
        pcl::transformPointCloud(*orientedSample, *outputCloud, sampleTransform);
    } else {
        pcl::transformPointCloud(*orientedSample, *outputCloud, sampleTransform);
    }
}

Eigen::Matrix4f icp_withNormals(
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_source_trans_normals,
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_target_normals,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source,
      PointCloud::Ptr& cloud_source_trans)
{
    // addtional refinement wit ICP
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
    
    icp.setInputSource ( cloud_source_trans_normals );
    icp.setInputTarget ( cloud_target_normals );
    
    // registration
    icp.align ( *cloud_source_trans_normals );
    
    if ( icp.hasConverged() ) {
        //pcl::transformPointCloud ( *cloud_source, *cloud_source_trans, icp.getFinalTransformation() );
        pcl::transformPointCloud ( *cloud_source_trans, *cloud_source_trans, icp.getFinalTransformation() );
        //std::cout << "converged next ICP" << std::endl;
    } else {
       std::cout << "Not converged." << std::endl;
    }

    return icp.getFinalTransformation();
}

int main (int argc, char** argv) {

    bool isVisualize = false;
    pcl::console::parse_argument (argc, argv, "-v", isVisualize);
    std::cout << "visualize correspondences: " << (isVisualize ? "true" : "false") << std::endl;

    std::vector<int> indices;
    pcl::VoxelGrid<pcl::PointXYZ> grid; //filter 
    pcl::VoxelGrid<pcl::PointXYZRGB> grid_color ; //filter color
    float radius, radius_target;
    float voxel_size = 0.005;
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Zero(); //set 4x4 matrix to 0
    ///
    PointCloudColor::Ptr color_source (new PointCloudColor);
    PointCloudColor::Ptr color_target (new PointCloudColor);
    PointCloudColor::Ptr color_source_filter (new PointCloudColor());
    ///
    PointCloudColor::Ptr color_source_trans (new PointCloudColor);
    PointCloud::Ptr cloud_source_trans ( new PointCloud );
    /// io clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_xyz ( new pcl::PointCloud<pcl::PointXYZ> () );
    PointCloudColor::Ptr raw_cloud_rgb (new PointCloudColor);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () ); // input to filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filter ( new pcl::PointCloud<pcl::PointXYZ> () ); // final filtered
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_target ( new pcl::PointCloud<pcl::PointXYZRGB> () );
    /// normals
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_trans_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> () ); //for icp with normals
    pcl::PointCloud<pcl::Normal>::Ptr source_normals ( new pcl::PointCloud<pcl::Normal> () );
    pcl::PointCloud<pcl::Normal>::Ptr target_normals ( new pcl::PointCloud<pcl::Normal> () );
    /// keypoints
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ ( new pcl::PointCloud<pcl::PointXYZ> () );
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ ( new pcl::PointCloud<pcl::PointXYZ> () );
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints ( new pcl::PointCloud<pcl::PointXYZI> () );
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints ( new pcl::PointCloud<pcl::PointXYZI> () );

  {
    // load cloud
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize (0.01f, 0.01f, 0.01f);
    //
    loadCloud ( argv[1], *raw_cloud_xyz );
    grid.setInputCloud (raw_cloud_xyz);
    grid.filter (*cloud_source);

    loadCloud ( argv[2], *raw_cloud_xyz ); 
    grid.setInputCloud (raw_cloud_xyz);
    grid.filter (*cloud_target);
    // load cloud -- color
    loadCloud_Color (argv[1], *color_source);
    loadCloud_Color (argv[2], *color_target);
    // normals
    addNormal ( cloud_source, source_normals, cloud_source_normals );
    addNormal ( cloud_target, target_normals, cloud_target_normals );
    addNormal ( cloud_source, source_normals, cloud_source_trans_normals ); // for icp with normals
  }
  
  {  
    Eigen::Vector4f max_pt, min_pt;
    pcl::getMinMax3D ( *cloud_source, min_pt, max_pt );
    radius = (max_pt - min_pt).norm() / 50; // fixed radius (scale) for detector/descriptor
    pcl::getMinMax3D ( *cloud_target, min_pt, max_pt );
    radius_target = (max_pt - min_pt).norm() / 50;
    std::cout << "radius scale | cloud source: " << radius << " , cloud target: " << radius_target << std::endl;
    
    //std::cout << std::to_string(inliers2) << std::endl;
    //std::cout << "previus target: " <<  cloud_target->points.size() << std::endl;
    //std::cout << "pos target: " <<  cloud_target->points.size() << std::endl;
    //rescaleClouds( cloud_source, cloud_target, false, true ); //aplly to cloud_target
  }

  {
        {
          std::cout << "Harris detector with normal: " << std::endl;
          pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI>::Ptr detector ( new pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI> () );
          detector->setNonMaxSupression ( true );
          detector->setRadius ( radius );
          detector->setRadiusSearch ( radius );
          detector->setMethod ( pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI>::CURVATURE ); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE 
          
          detector->setInputCloud ( cloud_source_normals );
          detector->setSearchSurface ( cloud_source_normals );
          detector->compute ( *source_keypoints );
          cout << "number of source keypoints found: " << source_keypoints->points.size() << endl;
          source_keypointsXYZ->points.resize( source_keypoints->points.size() );
          pcl::copyPointCloud( *source_keypoints, *source_keypointsXYZ );
          
          detector->setInputCloud ( cloud_target_normals );
          detector->setSearchSurface ( cloud_target_normals );
          detector->compute ( *target_keypoints );
          cout << "number of target keypoints found: " << target_keypoints->points.size() << endl;
          source_keypointsXYZ->points.resize( target_keypoints->points.size() );
          pcl::copyPointCloud( *target_keypoints, *target_keypointsXYZ );
        }
    
        #define useFPFH

        #ifdef useFPFH
            #define descriptorType pcl::FPFHSignature33
        #else
            #define descriptorType pcl::SHOT352
        #endif

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features (new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features (new pcl::PointCloud<pcl::FPFHSignature33>);
        
        { // descriptor "heavy"
          #ifdef useFPFH
              std::cout << "FPFHEstimationOMP" << std::endl;
              pcl::Feature<pcl::PointXYZ, descriptorType>::Ptr descriptor ( new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, descriptorType> () );
          #else
              std::cout << "SHOTEstimationOMP" << std::endl;
              pcl::Feature<pcl::PointXYZ, descriptorType>::Ptr descriptor ( new pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, descriptorType> () );
          #endif

          ///////////////////////////////////////////////////////
          descriptor->setSearchMethod (pcl::search::Search<pcl::PointXYZ>::Ptr ( new pcl::search::KdTree<pcl::PointXYZ>) );
          descriptor->setRadiusSearch (radius);
          pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, descriptorType>::Ptr feature_from_normals = std::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, descriptorType> > ( descriptor );
          ///////////////////////////////////////////////////////
          descriptor->setSearchSurface (cloud_source);
          descriptor->setInputCloud (source_keypointsXYZ);
          feature_from_normals->setInputNormals ( source_normals );
          descriptor->compute (*source_features);
          ///////////////////////////////////////////////////////
          descriptor->setSearchSurface (cloud_target);
          descriptor->setInputCloud (target_keypointsXYZ);
          feature_from_normals->setInputNormals ( target_normals );
          descriptor->compute (*target_features);
        }

      std::vector<int> correspondences;
      pcl::CorrespondencesPtr pCorrespondences ( new pcl::Correspondences );

      {
        std::cout << "Matching with Kd-tree..." << std::endl;
        correspondences.resize ( source_features->size() );
        pcl::KdTreeFLANN<descriptorType> search_tree;
        search_tree.setInputCloud ( target_features );
        
        std::vector<int> index(1);
        std::vector<float> L2_distance(1);

        for (int i = 0; i < source_features->size(); ++i) {

            correspondences[i] = -1; // -1 means no correspondence

            #ifdef useFPFH
                if ( isnan ( source_features->points[i].histogram[0] ) ) continue;
            #else
                if ( isnan ( source_features->points[i].descriptor[0] ) ) continue;
            #endif

            search_tree.nearestKSearch ( *source_features, i, 1, index, L2_distance );
            correspondences[i] = index[0];
        }
        // std::cout << "source_keypointsXYZ: " << source_keypointsXYZ->points.size() << " | target_keypointsXYZ: " << target_keypointsXYZ->points.size() << std::endl;
      }

        if(isVisualize){
            visualize_correspondences (cloud_source,  // funcion externa 1
                                      source_keypointsXYZ,
                                      cloud_target, 
                                      target_keypointsXYZ,
                                      correspondences);
        }

        {
          // Refining matching by filtering out wrong correspondence
          std::cout << "refining matching..." << std::endl;
          int nCorrespondence = 0;

          for (int i = 0; i < correspondences.size(); i++)
          if ( correspondences[i] >= 0 ) nCorrespondence++; // do not count "-1" in correspondences
          pCorrespondences->resize ( nCorrespondence );

          for (int i = 0, j = 0; i < correspondences.size(); i++){
            if ( correspondences[i] > 0 ){
              (*pCorrespondences)[j].index_query = i;
              (*pCorrespondences)[j].index_match = correspondences[i];
              j++;
            }
          }

          pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
          refine.setInputSource ( source_keypointsXYZ );
          refine.setInputTarget ( target_keypointsXYZ );
          refine.setInputCorrespondences ( pCorrespondences );
          refine.getCorrespondences ( *pCorrespondences );
        }
      
        if(isVisualize){
            visualize_correspondences (cloud_source,  // funcion externa 2
                                      source_keypointsXYZ,
                                      cloud_target, 
                                      target_keypointsXYZ,
                                      pCorrespondences);
        }
        
        { // registration ICP  
          std::cout << "------------------------------------------------------" << std::endl;
          std::cout << "pre-IterativeClosestPointWithNormals:" << std::endl << transformation << std::endl;

          transformation = icp_withNormals(cloud_source_trans_normals,
                                          cloud_target_normals,
                                          cloud_source,
                                          cloud_source_trans
                                          );

          std::cout << "pos-IterativeClosestPointWithNormals:" << std::endl << transformation << std::endl;
        
          // Estimating rigid transformation
          std::cout << "------------------------------------------------------" << std::endl;
          pcl::registration::TransformationEstimationSVDScale < pcl::PointXYZ, pcl::PointXYZ > estPtr;

          std::cout << "pre-estimateRigidTransformation SVD: " << std::endl << transformation << std::endl;

          estPtr.estimateRigidTransformation (*source_keypointsXYZ,
                                              *target_keypointsXYZ, 
                                              *pCorrespondences,
                                              transformation );
          
          std::cout << "pos-estimateRigidTransformation SVD: " << std::endl <<  transformation << std::endl;
          std::cout << "------------------------------------------------------" << std::endl;

          Eigen::Matrix3f R = transformation.matrix().topLeftCorner(3,3);
          float scale = std::sqrt( (R.transpose() * R).trace() / 3.0 );
          //transformation(3,3) = scale;
          //transformation.matrix().topLeftCorner(3,3) *= scale; 

          std::cout << "estimated scale: " << scale << std::endl;      
          std::cout << "estimated rotation: " << std::endl << R.matrix()  << std::endl;
          std::cout << "final transformation: " << std::endl << transformation  << std::endl;
        }
  }
  
  {
      // transformation
      pcl::transformPointCloud ( *cloud_source_normals, *cloud_source_trans_normals, transformation );  //for next icp
      pcl::transformPointCloud ( *color_source, *color_source_trans, transformation );  //color
      pcl::transformPointCloud ( *cloud_source, *cloud_source_trans, transformation );  //xyz
      
      {
        // visualization
        std::shared_ptr< pcl::visualization::PCLVisualizer > viewer ( new pcl::visualization::PCLVisualizer ("Vista 3D") );
        viewer->setBackgroundColor (0, 0, 0); //background=black
        viewer->setCameraPosition(0, 30, 0,    0, 0, 0,   0, 0, 1);
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( cloud_target, 255, 0, 0 ); //red
        viewer->addPointCloud<pcl::PointXYZ> ( cloud_target, target_color, "target");
        viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target" );
       
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_trans_color ( cloud_source_trans, 0, 0, 255 ); //blue
        viewer->addPointCloud<pcl::PointXYZ> ( cloud_source_trans, source_trans_color, "source trans" );
        viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source trans" );
        
        // orthographic (parallel) projection; same with pressing key 'o'
        viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection( 1 );
        viewer->resetCamera();
        viewer->spin ();
      }

      *cloud_target += *cloud_source_trans; // xyz transformation
      *color_target += *color_source_trans; // color transformation
      
      std::string output_file = "pair_icp.pcd";
      pcl::io::savePCDFile( output_file, *color_target, true ); // Binary format
      std::cerr << "Saved cloud: " << output_file << std::endl;
  }
  return(0);
}