// general
#include "ros/ros.h"
#include "iostream"

// messages
#include "std_msgs/String.h"
#include "body_tracker_msgs/Skeleton.h"
#include "ros_openpose/Frame.h"

// data
#include "procesador_rgb/TSkeleton_structs.h"

// other
#include <Eigen/Dense>  // Matrices, arrays, transformadas, etc

using namespace std;
using namespace Eigen;

// main class
class CSkeletonProcessor
{
  private: 
  
  vector<TSkeleton> esq_rgb = vector<TSkeleton>(4);
  TSkeleton_fe esq_fe;
  
  // subscribers
  ros::Subscriber sub_skeleton0, sub_skeleton1, sub_skeleton2, sub_skeleton3;
  ros::Subscriber sub_openpose;
  
  // auxiliary methods
  void copyJointData(const geometry_msgs::Point32& origen, Vector3f &destino)
  {
    destino[0] = origen.x;
    destino[1] = origen.y;
    destino[2] = origen.z;
  }
  
  void copiar_coordenadas(const ros_openpose::BodyPart& origen, Vector3f &destino)
  {
    destino[0] = origen.pixel.x;
    destino[1] = origen.pixel.y;
    destino[2] = origen.score;
  }
  
  void fuseSkeletons()
  {
    // TO-DO
  }

  void extrinsicCalibrateRGBD()
  {
    // TO-DO: Applies Horn method to compute the best alignment between RGBD cameras
  }

  public: 
  // constructor
  CSkeletonProcessor(ros::NodeHandle *nh) {
    // initalize subscriptions
    sub_skeleton0 = nh->subscribe<body_tracker_msgs::Skeleton>("body_tracker/skeleton", 1000, boost::bind(&CSkeletonProcessor::chatterCallback_rgb,this,_1,0)); //Se suscribe al topic body_tracker/skeleton
    sub_skeleton1 = nh->subscribe<body_tracker_msgs::Skeleton>("body_tracker/skeleton", 1000, boost::bind(&CSkeletonProcessor::chatterCallback_rgb,this,_1,1)); //Se suscribe al topic body_tracker/skeleton
    sub_skeleton2 = nh->subscribe<body_tracker_msgs::Skeleton>("body_tracker/skeleton", 1000, boost::bind(&CSkeletonProcessor::chatterCallback_rgb,this,_1,2)); //Se suscribe al topic body_tracker/skeleton
    sub_skeleton3 = nh->subscribe<body_tracker_msgs::Skeleton>("body_tracker/skeleton", 1000, boost::bind(&CSkeletonProcessor::chatterCallback_rgb,this,_1,3)); //Se suscribe al topic body_tracker/skeleton
    sub_openpose = nh->subscribe("frame", 1000, &CSkeletonProcessor::chatterCallback_fe, this); //Se suscribe al topic frame (esqueleto de la cámara ojo de pez)
  }

  // callbacks
  // this callback accepts the index of the camera to process
  void chatterCallback_rgb(const body_tracker_msgs::Skeleton::ConstPtr&  data, const int idx)
  {
    // DEBUG
    // ROS_INFO("join_position_head_x is: %f", data->joint_position_head.x);
    // ROS_INFO("join_position_head_y is: %f", data->joint_position_head.y);
    // ROS_INFO("join_position_head_z is: %f", data->joint_position_head.z);

    // copiar toda la información al esqueleto local
    copyJointData(data->joint_position_head, esq_rgb[idx].posicion_cabeza);
    copyJointData(data->joint_position_neck, esq_rgb[idx].posicion_cuello);
    copyJointData(data->joint_position_shoulder, esq_rgb[idx].posicion_hombro);
    copyJointData(data->joint_position_spine_top, esq_rgb[idx].posicion_columna_arriba);
    copyJointData(data->joint_position_spine_mid, esq_rgb[idx].posicion_columna_medio);
    copyJointData(data->joint_position_spine_bottom, esq_rgb[idx].posicion_columna_abajo);
    copyJointData(data->joint_position_left_shoulder, esq_rgb[idx].posicion_hombro_izquierdo);
    copyJointData(data->joint_position_left_elbow, esq_rgb[idx].posicion_codo_izquierdo);
    copyJointData(data->joint_position_left_hand, esq_rgb[idx].posicion_mano_izquierda);
    copyJointData(data->joint_position_right_shoulder, esq_rgb[idx].posicion_hombro_derecho);
    copyJointData(data->joint_position_right_elbow, esq_rgb[idx].posicion_codo_derecho);
    copyJointData(data->joint_position_right_hand, esq_rgb[idx].posicion_mano_derecha);

    esq_rgb[idx].status = data->tracking_status;

    // activar el flag de que el esqueleto esta listo para ser procesado
    // skeleton_ready[0] = true;
  }

  void chatterCallback_fe(const ros_openpose::Frame::ConstPtr&  data_fe)
  {
    // DEBUG
    ROS_INFO("position nose x is: %f", data_fe->persons[0].bodyParts[0].pixel.x);
    ROS_INFO("position nose y is: %f", data_fe->persons[0].bodyParts[0].pixel.y);

    // copiar toda la información al esqueleto local
    copiar_coordenadas(data_fe->persons[0].bodyParts[0], esq_fe.posicion_cabeza);
    copiar_coordenadas(data_fe->persons[0].bodyParts[1], esq_fe.posicion_cuello);
    copiar_coordenadas(data_fe->persons[0].bodyParts[8], esq_fe.posicion_cadera);

    copiar_coordenadas(data_fe->persons[0].bodyParts[16], esq_fe.posicion_ojo_izquierdo);
    copiar_coordenadas(data_fe->persons[0].bodyParts[18], esq_fe.posicion_oreja_izquierda);
    copiar_coordenadas(data_fe->persons[0].bodyParts[15], esq_fe.posicion_ojo_derecho);
    copiar_coordenadas(data_fe->persons[0].bodyParts[17], esq_fe.posicion_oreja_derecha);

    copiar_coordenadas(data_fe->persons[0].bodyParts[5], esq_fe.posicion_hombro_izquierdo);
    copiar_coordenadas(data_fe->persons[0].bodyParts[6], esq_fe.posicion_codo_izquierdo);
    copiar_coordenadas(data_fe->persons[0].bodyParts[7], esq_fe.posicion_mano_izquierda);
    copiar_coordenadas(data_fe->persons[0].bodyParts[12], esq_fe.posicion_cadera_izquierda);
    copiar_coordenadas(data_fe->persons[0].bodyParts[13], esq_fe.posicion_rodilla_izquierda);
    copiar_coordenadas(data_fe->persons[0].bodyParts[14], esq_fe.posicion_pie_izquierdo_talon);
    copiar_coordenadas(data_fe->persons[0].bodyParts[19], esq_fe.posicion_pie_izquierdo_punta);

    copiar_coordenadas(data_fe->persons[0].bodyParts[2], esq_fe.posicion_hombro_derecho);
    copiar_coordenadas(data_fe->persons[0].bodyParts[3], esq_fe.posicion_codo_derecho);
    copiar_coordenadas(data_fe->persons[0].bodyParts[4], esq_fe.posicion_mano_derecha);
    copiar_coordenadas(data_fe->persons[0].bodyParts[9], esq_fe.posicion_cadera_derecha);
    copiar_coordenadas(data_fe->persons[0].bodyParts[10], esq_fe.posicion_rodilla_derecha);
    copiar_coordenadas(data_fe->persons[0].bodyParts[11], esq_fe.posicion_pie_derecha_talon);
    copiar_coordenadas(data_fe->persons[0].bodyParts[22], esq_fe.posicion_pie_derecha_punta);
  }

};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "skeleton_processor"); //Inicializa el nodo
  ros::NodeHandle nh;
  CSkeletonProcessor sp = CSkeletonProcessor(&nh);
  ros::spin();

  return 0;
}
