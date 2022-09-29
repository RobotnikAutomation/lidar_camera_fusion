#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <math.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <armadillo>

#include <chrono>

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZI> LC_PointCloud;

//Publisher
ros::Publisher pcOnimg_pub;
ros::Publisher pc_pub;


float maxlen =100.0;       //maxima distancia del lidar
float minlen = 0.01;     //minima distancia del lidar
float max_FOV = 6.28;    // en radianes angulo maximo de vista de la camara
float min_FOV = 0.0;    // en radianes angulo minimo de vista de la camara

/// parametros para convertir nube de puntos en imagen
float angular_resolution_x =0.5f;
float angular_resolution_y = 2.1f;
float angular_correction = 1.0;
float max_angle_width= 360.0f;
float max_angle_height = 180.0f;
float z_max = 100.0f;
float z_min = 100.0f;

float max_depth =100.0;
float min_depth = 8.0;

float interpol_value = 20.0;

// input topics
std::string imgTopic = "/camera/color/image_raw";
std::string pcTopic = "/velodyne_points";

// frames
std::string lidarFrame = "robot_top_3d_laser_link";
std::string cameraFrame = "robot_front_ptz_camera_frame_link";
std::string inspectionPointFrame = "robot_inspection_point";

//matrix calibration lidar and camera

Eigen::MatrixXf Tlc(3,1); // translation matrix lidar-camera
Eigen::MatrixXf Rlc(3,3); // rotation matrix lidar-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix

// range image parametros
boost::shared_ptr<pcl::RangeImageSpherical> rangeImage;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;



///////////////////////////////////////callback



void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& in_pc2 , const ImageConstPtr& in_image)
{
    int height = in_image->height;
    int width = in_image->width;

    cv_bridge::CvImagePtr cv_ptr , color_pcl;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::BGR8);
          color_pcl = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

  //Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*in_pc2,pcl_pc2);
  LC_PointCloud::Ptr msg_pointCloud(new LC_PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*msg_pointCloud);
  ///

  ////// filter point cloud
  if (msg_pointCloud == NULL) return;

  LC_PointCloud::Ptr cloud_in (new LC_PointCloud);
  //PointCloud::Ptr cloud_filter (new PointCloud);
  LC_PointCloud::Ptr cloud_out (new LC_PointCloud);

  //PointCloud::Ptr cloud_aux (new PointCloud);
 // pcl::PointXYZI point_aux;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg_pointCloud, *cloud_in, indices);

  for (int i = 0; i < (int) cloud_in->points.size(); i++)
  {
      double distance = sqrt(cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y);
      if(distance<minlen || distance>maxlen)
       continue;
      cloud_out->push_back(cloud_in->points[i]);

  }


  //                                                  point cloud to image

  //============================================================================================================
  //============================================================================================================

  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  rangeImage->pcl::RangeImage::createFromPointCloud(*cloud_out, pcl::deg2rad(angular_resolution_x), pcl::deg2rad(angular_resolution_y),
                                       pcl::deg2rad(max_angle_width), pcl::deg2rad(max_angle_height),
                                       sensorPose, coordinate_frame, 0.0f, 0.0f, 0);



  int cols_img = rangeImage->width;
  int rows_img = rangeImage->height;


  arma::mat Z;  // interpolation de la imagen
  arma::mat Zz; // interpolation de las alturas de la imagen

  Z.zeros(rows_img,cols_img);
  Zz.zeros(rows_img,cols_img);

  Eigen::MatrixXf ZZei (rows_img,cols_img);

  for (int i=0; i< cols_img; ++i)
      for (int j=0; j<rows_img ; ++j)
      {
        float r =  rangeImage->getPoint(i, j).range;
        float zz = rangeImage->getPoint(i, j).z;

       // Eigen::Vector3f tmp_point;
        //rangeImage->calculate3DPoint (float(i), float(j), r, tmp_point);
        if(std::isinf(r) || r<minlen || r>maxlen || std::isnan(zz)){
            continue;
        }
        Z.at(j,i) = r;
        Zz.at(j,i) = zz;
        //ZZei(j,i)=tmp_point[2];


        //point_aux.x = tmp_point[0];
        //point_aux.y = tmp_point[1];
        //point_aux.z = tmp_point[2];

       // cloud_aux->push_back(point_aux);



        //std::cout<<"i: "<<i<<" Z.getpoint: "<<zz<<" tmpPoint: "<<tmp_point<<std::endl;

      }

  ////////////////////////////////////////////// interpolation
  //============================================================================================================

  arma::vec X = arma::regspace(1, Z.n_cols);  // X = horizontal spacing
  arma::vec Y = arma::regspace(1, Z.n_rows);  // Y = vertical spacing



  arma::vec XI = arma:: regspace(X.min(), 1.0, X.max()); // magnify by approx 2
  arma::vec YI = arma::regspace(Y.min(), 1.0/interpol_value, Y.max()); //


  arma::mat ZI_near;
  arma::mat ZI;
  arma::mat ZzI;

  arma::interp2(X, Y, Z, XI, YI, ZI,"lineal");
  arma::interp2(X, Y, Zz, XI, YI, ZzI,"lineal");

  //===========================================fin filtrado por imagen=================================================
  /////////////////////////////

  // reconstruccion de imagen a nube 3D
  //============================================================================================================


  LC_PointCloud::Ptr point_cloud (new LC_PointCloud);
  LC_PointCloud::Ptr cloud (new LC_PointCloud);
  point_cloud->width = ZI.n_cols;
  point_cloud->height = ZI.n_rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);

  arma::mat Zout = ZI;

  Eigen::MatrixXf RTlc(4,4); // translation matrix lidar-camera
  Eigen::MatrixXf Rroll(3,3), Rpitch(3,3), Ryaw(3,3), Rfinal(3,3);
  tf::TransformListener listener;
  tf::StampedTransform lidar2camera_transform;

  try {
      listener.waitForTransform(lidarFrame, cameraFrame, ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform(lidarFrame, cameraFrame, ros::Time(0), lidar2camera_transform);
      tf::Quaternion q = lidar2camera_transform.getRotation();
      tf::Matrix3x3 m(q);
            
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      
      Rroll << cos(roll),   sin(roll), 0, 
               -sin(roll),  cos(roll), 0,
               0,           0,         1; 

      Rpitch << 1,          0,           0 ,
                0,          cos(pitch), -sin(pitch), 
                0,           sin(pitch), cos(pitch); 

      Ryaw << cos(yaw)     , 0    , sin(yaw), 
              0           , 1    , 0,        
              -sin(yaw)   , 0    , cos(yaw);
      
      Rfinal = Rroll*Rpitch*Ryaw;

      RTlc<<  Rfinal(0,0),  Rfinal(0,1), Rfinal(0,2), 0,
              Rfinal(1,0),  Rfinal(1,1), Rfinal(1,2), 0,
              Rfinal(2,0),  Rfinal(2,1), Rfinal(2,2), 0,
              0             , 0             , 0        , 1;

  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
  }
  
  //////////////////filtrado de elementos interpolados con el fondo
  for (uint i=0; i< ZI.n_rows; i+=1)
   {
      for (uint j=0; j<ZI.n_cols ; j+=1)
      {
       if((ZI(i,j)== 0 ))
       {
        if(i+interpol_value<ZI.n_rows)
          for (int k=1; k<= interpol_value; k+=1)
            Zout(i+k,j)=0;
        if(i>interpol_value)
          for (int k=1; k<= interpol_value; k+=1)
            Zout(i-k,j)=0;
        }
      }
    }

  ///////// imagen de rango a nube de puntos
  int num_pc = 0;
  for (uint i=0; i< ZI.n_rows - interpol_value; i+=1)
   {
      for (uint j=0; j<ZI.n_cols ; j+=1)
      {

        float ang = M_PI-((2.0 * M_PI * j )/(ZI.n_cols));
        if (ang < min_FOV-M_PI/2.0|| ang > max_FOV - M_PI/2.0)
          continue;

        if(!(Zout(i,j)== 0 ))
        {
          float pc_modulo = Zout(i,j);
          float pc_x = sqrt(pow(pc_modulo,2)- pow(ZzI(i,j),2))* cos(ang);
          float pc_y = sqrt(pow(pc_modulo,2)- pow(ZzI(i,j),2))* sin(ang);

          float ang_x_lidar = angular_correction*M_PI/180.0;

          Eigen::MatrixXf Lidar_matrix(3,3); //matrix  transformation between lidar and range image. It rotates the angles that it has of error with respect to the ground
          Eigen::MatrixXf result(3,1);

          Lidar_matrix <<   cos(ang_x_lidar) ,0                ,sin(ang_x_lidar),
                            0                ,1                ,0,
                            -sin(ang_x_lidar),0                ,cos(ang_x_lidar) ;

          result << pc_x,
                    pc_y,
                    ZzI(i,j);

          result = Lidar_matrix*result;  // rotacion en eje X para correccion

          point_cloud->points[num_pc].x = result(0);
          point_cloud->points[num_pc].y = result(1);
          point_cloud->points[num_pc].z = result(2);

          cloud->push_back(point_cloud->points[num_pc]);

          num_pc++;
        }
      }
   }

  //============================================================================================================

   LC_PointCloud::Ptr P_out (new LC_PointCloud);

   //filremove noise of point cloud
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50.0);
  sor.setStddevMulThresh (1.0);
  sor.filter (*P_out);*/

  // dowsmapling
  /*pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*P_out);*/


  P_out = cloud;

  // std::cout<<RTlc<<std::endl;

  int size_inter_Lidar = (int) P_out->points.size();

  Eigen::MatrixXf Lidar_camera(3,size_inter_Lidar);
  Eigen::MatrixXf Lidar_cam(3,1);
  Eigen::MatrixXf pc_matrix(4,1);
  Eigen::MatrixXf pointCloud_matrix(4,size_inter_Lidar);

  unsigned int cols = in_image->width;
  unsigned int rows = in_image->height;

  uint px_data = 0; uint py_data = 0;


  pcl::PointXYZRGB point;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_color (new pcl::PointCloud<pcl::PointXYZRGB>);

   //P_out = cloud_out;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  bool detected_center = false;
  geometry_msgs::PointStamped in_point, transformed_pose;

  for (int i = 0; i < size_inter_Lidar; i++)
  {
      pc_matrix(0,0) =  -P_out->points[i].y;
      pc_matrix(1,0) =  -P_out->points[i].z;
      pc_matrix(2,0) =  P_out->points[i].x;
      pc_matrix(3,0) = 1.0;

      Lidar_cam = Mc * (RTlc * pc_matrix);

      px_data = (int)(Lidar_cam(0,0)/Lidar_cam(2,0));
      py_data = (int)(Lidar_cam(1,0)/Lidar_cam(2,0));

      int right_limit = width/2 + 10;
      int left_limit = width/2 - 10;
      int upper_limit = height/2 + 10;
      int bottom_limit = height/2 - 10;

      if(px_data<0.0 || px_data>=cols || py_data<0.0 || py_data>=rows)
        continue;

      if(!detected_center && px_data>=left_limit && px_data<=right_limit && py_data>=bottom_limit && py_data<=upper_limit)
      {
        // in_point.header.frame_id = "robot_top_3d_laser_link";
        // in_point.point.x = P_out->points[i].x;
        // in_point.point.y = P_out->points[i].y;
        // in_point.point.z = P_out->points[i].z;
        // listener.transformPoint("robot_odom", in_point, transformed_pose);

        transform.setOrigin( tf::Vector3(P_out->points[i].x, P_out->points[i].y, P_out->points[i].z) );
        // transform.setOrigin( tf::Vector3(transformed_pose.point.x, transformed_pose.point.y, transformed_pose.point.z) );
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), lidarFrame, inspectionPointFrame));
        detected_center = true;
      }

      int color_dis_x = (int)(255*((P_out->points[i].x)/maxlen));
      int color_dis_z = (int)(255*((P_out->points[i].x)/10.0));
      if(color_dis_z>255)
          color_dis_z = 255;


      //point cloud con color
      cv::Vec3b & color = color_pcl->image.at<cv::Vec3b>(py_data,px_data);

      // point.x = transformed_pose.point.x;
      // point.y = transformed_pose.point.y;
      // point.z = transformed_pose.point.z;
      point.x = P_out->points[i].x;
      point.y = P_out->points[i].y;
      point.z = P_out->points[i].z;


      point.r = (int)color[2];
      point.g = (int)color[1];
      point.b = (int)color[0];


      pc_color->points.push_back(point);

      cv::circle(cv_ptr->image, cv::Point(px_data, py_data), 1, CV_RGB(255-color_dis_x,(int)(color_dis_z),color_dis_x),cv::FILLED);

    }
    pc_color->is_dense = true;
    pc_color->width = (int) pc_color->points.size();
    pc_color->height = 1;
    pc_color->header.frame_id = lidarFrame;

  pcOnimg_pub.publish(cv_ptr->toImageMsg());
  pc_pub.publish (pc_color);

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pontCloudOntImage");
  ros::NodeHandle nh;


  /// Load Parameters

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/max_ang_FOV", max_FOV);
  nh.getParam("/min_ang_FOV", min_FOV);
  nh.getParam("/pcTopic", pcTopic);
  nh.getParam("/imgTopic", imgTopic);
  nh.getParam("/lidar_frame", lidarFrame)
  nh.getParam("/camera_frame", cameraFrame)
  nh.getParam("/inspection_point_frame", inspectionPointFrame)

  nh.getParam("/x_resolution", angular_resolution_x);
  nh.getParam("/y_interpolation", interpol_value);

  nh.getParam("/ang_Y_resolution", angular_resolution_y);
  nh.getParam("/angular_correction", angular_correction);


  XmlRpc::XmlRpcValue param;

  nh.getParam("/matrix_file/tlc", param);
  Tlc <<  (double)param[0]
         ,(double)param[1]
         ,(double)param[2];

  nh.getParam("/matrix_file/rlc", param);

  tf::Matrix3x3 m;
  m.setRPY(param[0], param[1], param[2]);

  Rlc <<  (double)param[0] ,(double)param[1] ,(double)param[2]
         ,(double)param[3] ,(double)param[4] ,(double)param[5]
         ,(double)param[6] ,(double)param[7] ,(double)param[8];

  nh.getParam("/matrix_file/camera_matrix", param);

  Mc  <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3]
         ,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7]
         ,(double)param[8] ,(double)param[9] ,(double)param[10],(double)param[11];

  message_filters::Subscriber<PointCloud2> pc_sub(nh, pcTopic , 1);
  message_filters::Subscriber<Image> img_sub(nh, imgTopic, 1);

  typedef sync_policies::ApproximateTime<PointCloud2, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, img_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  pcOnimg_pub = nh.advertise<sensor_msgs::Image>("/pcOnImage_image", 1);
  rangeImage = boost::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);

  pc_pub = nh.advertise<LC_PointCloud> ("/points2", 1);

  ros::spin();
}
