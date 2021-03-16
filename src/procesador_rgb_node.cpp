#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "std_msgs/Float32.h"
#include "body_tracker_msgs/Skeleton.h"
#include "iostream"
#include <Eigen/Dense>  //Matrices, arrays, transformadas, etc
#include "ros_openpose/Frame.h"
using namespace std;
using namespace Eigen;


struct TSkeleton
{
  // posiciones de las articulaciones
  Vector3f posicion_cabeza;
  Vector3f posicion_cuello;
  Vector3f posicion_hombro;
  Vector3f posicion_columna_arriba;
  Vector3f posicion_columna_medio;
  Vector3f posicion_columna_abajo;

  Vector3f posicion_hombro_izquierdo;
  Vector3f posicion_codo_izquierdo;
  Vector3f posicion_mano_izquierda;

  Vector3f posicion_hombro_derecho;
  Vector3f posicion_codo_derecho;
  Vector3f posicion_mano_derecha;

  // estado de la estimacion segun el nodo de la camara
  int Status;

  // transformacion 3D entre este esqueleto y el de referencia (se rellena al inicio)
  Matrix4f transformacion;
};

struct TSkeleton_fe
{
  //Son vectores de tres posiciones, la primera la coordenada x, la segunda la y y la tercera la precisión
  Vector3f posicion_cabeza;
  Vector3f posicion_cuello;
  Vector3f posicion_cadera;

  Vector3f posicion_ojo_izquierdo;
  Vector3f posicion_oreja_izquierda;

  Vector3f posicion_ojo_derecho;
  Vector3f posicion_oreja_derecha;


  Vector3f posicion_hombro_izquierdo;
  Vector3f posicion_codo_izquierdo;
  Vector3f posicion_mano_izquierda;
  Vector3f posicion_cadera_izquierda;
  Vector3f posicion_rodilla_izquierda;
  Vector3f posicion_pie_izquierdo_talon;
  Vector3f posicion_pie_izquierdo_punta;

  Vector3f posicion_hombro_derecho;
  Vector3f posicion_codo_derecho;
  Vector3f posicion_mano_derecha;
  Vector3f posicion_cadera_derecha;
  Vector3f posicion_rodilla_derecha;
  Vector3f posicion_pie_derecha_talon;
  Vector3f posicion_pie_derecha_punta;

};

vector<TSkeleton> esq_rgb(4);
TSkeleton_fe esq_fe;

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

void chatterCallback_rgb(const body_tracker_msgs::Skeleton::ConstPtr&  data)
{
  // DEBUG
  // ROS_INFO("join_position_head_x is: %f", data->joint_position_head.x);
  // ROS_INFO("join_position_head_y is: %f", data->joint_position_head.y);
  // ROS_INFO("join_position_head_z is: %f", data->joint_position_head.z);

  // copiar toda la información al esqueleto local
  copyJointData(data->joint_position_head, esq_rgb[0].posicion_cabeza);
  copyJointData(data->joint_position_neck, esq_rgb[0].posicion_cuello);
  copyJointData(data->joint_position_shoulder, esq_rgb[0].posicion_hombro);
  copyJointData(data->joint_position_spine_top, esq_rgb[0].posicion_columna_arriba);
  copyJointData(data->joint_position_spine_mid, esq_rgb[0].posicion_columna_medio);
  copyJointData(data->joint_position_spine_bottom, esq_rgb[0].posicion_columna_abajo);
  copyJointData(data->joint_position_left_shoulder, esq_rgb[0].posicion_hombro_izquierdo);
  copyJointData(data->joint_position_left_elbow, esq_rgb[0].posicion_codo_izquierdo);
  copyJointData(data->joint_position_left_hand, esq_rgb[0].posicion_mano_izquierda);
  copyJointData(data->joint_position_right_shoulder, esq_rgb[0].posicion_hombro_derecho);
  copyJointData(data->joint_position_right_elbow, esq_rgb[0].posicion_codo_derecho);
  copyJointData(data->joint_position_right_hand, esq_rgb[0].posicion_mano_derecha);

  // activar el flag de que el esqueleto esta listo para ser procesado
  // skeleton_ready[0] = true;

  /** /
    esq_rgb[0].posicion_cabeza[0] = data->joint_position_head.x;
    esq_rgb[0].posicion_cabeza[1] = data->joint_position_head.y;
    esq_rgb[0].posicion_cabeza[2] = data->joint_position_head.z;

    esq_rgb[0].posicion_cuello[0] = data->joint_position_neck.x;
    esq_rgb[0].posicion_cuello[1] = data->joint_position_neck.y;
    esq_rgb[0].posicion_cuello[2] = data->joint_position_neck.z;

    esq_rgb[0].posicion_hombro[0] = data->joint_position_shoulder.x;
    esq_rgb[0].posicion_hombro[1] = data->joint_position_shoulder.y;
    esq_rgb[0].posicion_hombro[2] = data->joint_position_shoulder.z;

    esq_rgb[0].posicion_columna_arriba[0] = data->joint_position_spine_top.x;
    esq_rgb[0].posicion_columna_arriba[1] = data->joint_position_spine_top.y;
    esq_rgb[0].posicion_columna_arriba[2] = data->joint_position_spine_top.z;

    esq_rgb[0].posicion_columna_medio[0] = data->joint_position_spine_mid.x;
    esq_rgb[0].posicion_columna_medio[1] = data->joint_position_spine_mid.y;
    esq_rgb[0].posicion_columna_medio[2] = data->joint_position_spine_mid.z;

    esq_rgb[0].posicion_columna_abajo[0] = data->joint_position_spine_bottom.x;
    esq_rgb[0].posicion_columna_abajo[1] = data->joint_position_spine_bottom.y;
    esq_rgb[0].posicion_columna_abajo[2] = data->joint_position_spine_bottom.z;

    esq_rgb[0].posicion_hombro_izquierdo[0] = data->joint_position_left_shoulder.x;
    esq_rgb[0].posicion_hombro_izquierdo[1] = data->joint_position_left_shoulder.y;
    esq_rgb[0].posicion_hombro_izquierdo[2] = data->joint_position_left_shoulder.z;

    esq_rgb[0].posicion_codo_izquierdo[0] = data->joint_position_left_elbow.x;
    esq_rgb[0].posicion_codo_izquierdo[1] = data->joint_position_left_elbow.y;
    esq_rgb[0].posicion_codo_izquierdo[2] = data->joint_position_left_elbow.z;

    esq_rgb[0].posicion_mano_izquierda[0] = data->joint_position_left_hand.x;
    esq_rgb[0].posicion_mano_izquierda[1] = data->joint_position_left_hand.y;
    esq_rgb[0].posicion_mano_izquierda[2] = data->joint_position_left_hand.z;

    esq_rgb[0].posicion_hombro_derecho[0] = data->joint_position_right_shoulder.x;
    esq_rgb[0].posicion_hombro_derecho[1] = data->joint_position_right_shoulder.y;
    esq_rgb[0].posicion_hombro_derecho[2] = data->joint_position_right_shoulder.z;

    esq_rgb[0].posicion_codo_derecho[0] = data->joint_position_right_elbow.x;
    esq_rgb[0].posicion_codo_derecho[1] = data->joint_position_right_elbow.y;
    esq_rgb[0].posicion_codo_derecho[2] = data->joint_position_right_elbow.z;

    esq_rgb[0].posicion_mano_derecha[0] = data->joint_position_right_hand.x;
    esq_rgb[0].posicion_mano_derecha[1] = data->joint_position_right_hand.y;
    esq_rgb[0].posicion_mano_derecha[2] = data->joint_position_right_hand.z;
  /**/
    esq_rgb[0].Status = data->tracking_status;

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

// void fuseSkeletons(std::vector<TSkeleton> esq_rgb)
// {


// }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener"); //Inicializa el nodo
  ros::NodeHandle n;

 
  
  //-------------------------------------------
  //Tenemos que incluir una matriz 3x3 que guarde la posición de la cámara respecto a una de referencia
  //Hay que crear una clase que se encargue de leer los esqueletos y procesarlos
  //------------------------------------
  ros::Subscriber sub_skeleton = n.subscribe("body_tracker/skeleton", 1000, chatterCallback_rgb); //Se suscribe al topic body_tracker/skeleton
  ros::Subscriber sub_openpose = n.subscribe("frame", 1000, chatterCallback_fe); //Se suscribe al topic frame (esqueleto de la cámara ojo de pez)

  ros::spin();


  return 0;
}
