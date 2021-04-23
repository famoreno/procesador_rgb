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
#include <Eigen/Dense>        // matrices, arrays, transformadas, etc
#include <Eigen/Eigenvalues>  // for eigenvectors and eigenvalues
#include <Eigen/Geometry>     // for quaternions
#include <stdio.h>
// #include "conio.h"
#include <math.h>
#define MAX_LAPSE 0.2

using namespace std;
using namespace Eigen;

// main class
class CSkeletonProcessor
{
  private: 
  
  vector<TSkeleton> esq_rgb = vector<TSkeleton>(4);
  TSkeleton_fe esq_fe;
  
  vector<TSkeleton> esq_rgb_ref = vector<TSkeleton>(4);
  TSkeleton esq_fused;

  vector<bool> skeleton_ready = vector<bool>(4);

  FILE* doc;

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
  
  void camaras_ref()
  {
    for (int i = 0; i<=3; i++) {
      if( !skeleton_ready[i] )
      continue;

      transformar_coordenadas(i);
    }
  }

  void transformar_coordenadas(int i)
  {
    esq_rgb_ref[i].posicion_cabeza = esq_rgb[i].rotacion * esq_rgb[i].posicion_cabeza + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_cuello = esq_rgb[i].rotacion * esq_rgb[i].posicion_cuello + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_hombro = esq_rgb[i].rotacion * esq_rgb[i].posicion_hombro + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_columna_arriba = esq_rgb[i].rotacion * esq_rgb[i].posicion_columna_arriba + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_columna_medio = esq_rgb[i].rotacion * esq_rgb[i].posicion_columna_medio + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_columna_abajo = esq_rgb[i].rotacion * esq_rgb[i].posicion_columna_abajo + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_hombro_izquierdo = esq_rgb[i].rotacion * esq_rgb[i].posicion_hombro_izquierdo + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_codo_izquierdo = esq_rgb[i].rotacion * esq_rgb[i].posicion_codo_izquierdo + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_mano_izquierda = esq_rgb[i].rotacion * esq_rgb[i].posicion_mano_izquierda + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_hombro_derecho = esq_rgb[i].rotacion * esq_rgb[i].posicion_hombro_derecho + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_codo_derecho = esq_rgb[i].rotacion * esq_rgb[i].posicion_codo_derecho + esq_rgb[i].traslacion;
    esq_rgb_ref[i].posicion_mano_derecha = esq_rgb[i].rotacion * esq_rgb[i].posicion_mano_derecha + esq_rgb[i].traslacion;
  }


  void copiar_esqueleto(std::vector<Vector3f> destino, int idx)
  {
    for(unsigned char i = 0; i < 3; i++) destino[0][i]  = esq_rgb[idx].posicion_cabeza[i];
    for(unsigned char i = 0; i < 3; i++) destino[1][i]  = esq_rgb[idx].posicion_cuello[i];
    for(unsigned char i = 0; i < 3; i++) destino[2][i]  = esq_rgb[idx].posicion_hombro[i];
    for(unsigned char i = 0; i < 3; i++) destino[3][i]  = esq_rgb[idx].posicion_columna_arriba[i];
    for(unsigned char i = 0; i < 3; i++) destino[4][i]  = esq_rgb[idx].posicion_columna_medio[i];
    for(unsigned char i = 0; i < 3; i++) destino[5][i]  = esq_rgb[idx].posicion_columna_abajo[i];
    for(unsigned char i = 0; i < 3; i++) destino[6][i]  = esq_rgb[idx].posicion_hombro_izquierdo[i];
    for(unsigned char i = 0; i < 3; i++) destino[7][i]  = esq_rgb[idx].posicion_codo_izquierdo[i];
    for(unsigned char i = 0; i < 3; i++) destino[8][i]  = esq_rgb[idx].posicion_mano_izquierda[i];
    for(unsigned char i = 0; i < 3; i++) destino[9][i]  = esq_rgb[idx].posicion_hombro_derecho[i];
    for(unsigned char i = 0; i < 3; i++) destino[10][i] = esq_rgb[idx].posicion_codo_derecho[i];
    for(unsigned char i = 0; i < 3; i++) destino[11][i] = esq_rgb[idx].posicion_mano_derecha[i];
  }

  void calcular_centroide(Vector3f & ct_destino, int idx)
  {
    for( unsigned char i = 0; i < 3; i++ ) 
    {
      ct_destino[i] = (1/12)*(esq_rgb[idx].posicion_cabeza[i] + esq_rgb[idx].posicion_cuello[i] + esq_rgb[idx].posicion_hombro[i] + 
                              esq_rgb[idx].posicion_columna_arriba[i] + esq_rgb[idx].posicion_columna_medio[i] + esq_rgb[idx].posicion_columna_abajo[i] +
                              esq_rgb[idx].posicion_hombro_izquierdo[i] + esq_rgb[idx].posicion_codo_izquierdo[i] + esq_rgb[idx].posicion_mano_izquierda[i] + 
                              esq_rgb[idx].posicion_hombro_derecho[i] + esq_rgb[idx].posicion_codo_derecho[i] +esq_rgb[idx].posicion_mano_derecha[i]);
    }
  }

  void extrinsicCalibrateRGBD(const int idx)
  {
    // copiar esqueletos a un vector: 'point_this' y 'point_other'
    vector<Vector3f> points_this(12), points_other(12);

    copiar_esqueleto(points_this, 0);
    copiar_esqueleto(points_other, 1);

    // paso 1: calcular centroides
    Vector3f ct_this, ct_others;
    calcular_centroide(ct_this, 0);
    calcular_centroide(ct_others, 1);

    // paso 2: restar el centroide a los puntos y calcular los componentes de S
	  Matrix3f S;	// Zeroed by default
    for (size_t i = 0; i < points_other.size(); i++)
    {
      points_this[i] -= ct_this;
      points_other[i] -= ct_others;

      S(0, 0) += points_other[i][0] * points_this[i][0];
      S(0, 1) += points_other[i][0] * points_this[i][1];
      S(0, 2) += points_other[i][0] * points_this[i][2];

      S(1, 0) += points_other[i][1] * points_this[i][0];
      S(1, 1) += points_other[i][1] * points_this[i][1];
      S(1, 2) += points_other[i][1] * points_this[i][2];

      S(2, 0) += points_other[i][2] * points_this[i][0];
      S(2, 1) += points_other[i][2] * points_this[i][1];
      S(2, 2) += points_other[i][2] * points_this[i][2];
    }

    // paso 3: construye la matriz N
    Matrix4f N;	// Zeroed by default
    N(0, 0) = S(0, 0) + S(1, 1) + S(2, 2);
    N(0, 1) = S(1, 2) - S(2, 1);
    N(0, 2) = S(2, 0) - S(0, 2);
    N(0, 3) = S(0, 1) - S(1, 0);

    N(1, 0) = N(0, 1);
    N(1, 1) = S(0, 0) - S(1, 1) - S(2, 2);
    N(1, 2) = S(0, 1) + S(1, 0);
    N(1, 3) = S(2, 0) + S(0, 2);

    N(2, 0) = N(0, 2);
    N(2, 1) = N(1, 2);
    N(2, 2) = -S(0, 0) + S(1, 1) - S(2, 2);
    N(2, 3) = S(1, 2) + S(2, 1);

    N(3, 0) = N(0, 3);
    N(3, 1) = N(1, 3);
    N(3, 2) = N(2, 3);
    N(3, 3) = -S(0, 0) - S(1, 1) + S(2, 2);

	// paso 4: calcular los autovectores de la matriz N (q es el quaternion de rotacion y es igual al autovector correspondiente al mayor autovalor)
	// matrix (last column in Z)
  Matrix4f Z;

  // la matriz N es simetrica --> selfadjointeigensolver
  SelfAdjointEigenSolver<Matrix4f> es(N);
  Vector4f v = es.eigenvectors().col(3); // get the largest 'eigenvector', it should normalized to one
  
	// ASSERTDEB_(
	// 	fabs(
	// 		sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]) - 1.0) <
	// 	0.1);

	// Make q_r > 0
	if (v[0] < 0)
	{
		v[0] *= -1;
		v[1] *= -1;
		v[2] *= -1;
		v[3] *= -1;
	}

  // paso 5: rotar el centroide de "others" con la rotacion en forma de matriz y restarlo del "ct_this"
  Quaternionf q(v);
  Matrix3f rotM = q.toRotationMatrix();
  Vector3f n_ct_others = rotM*ct_others;
  Vector3f t = ct_this - n_ct_others;

  // save output
  esq_rgb[idx].rotacion = rotM;
  esq_rgb[idx].traslacion = t;

 }  // extrinsicCalibration

  public: 
  // constructor
  CSkeletonProcessor(ros::NodeHandle *nh) {
    // initalize subscriptions
    sub_skeleton0 = nh->subscribe<body_tracker_msgs::Skeleton>("body_tracker_sensor0/skeleton", 1000, boost::bind(&CSkeletonProcessor::chatterCallback_rgb,this,_1,0)); //Se suscribe al topic body_tracker/skeleton
    sub_skeleton1 = nh->subscribe<body_tracker_msgs::Skeleton>("body_tracker_sensor1/skeleton", 1000, boost::bind(&CSkeletonProcessor::chatterCallback_rgb,this,_1,1)); //Se suscribe al topic body_tracker/skeleton
    sub_skeleton2 = nh->subscribe<body_tracker_msgs::Skeleton>("body_tracker_sensor2/skeleton", 1000, boost::bind(&CSkeletonProcessor::chatterCallback_rgb,this,_1,2)); //Se suscribe al topic body_tracker/skeleton
    sub_skeleton3 = nh->subscribe<body_tracker_msgs::Skeleton>("body_tracker/skeleton", 1000, boost::bind(&CSkeletonProcessor::chatterCallback_rgb,this,_1,3)); //Se suscribe al topic body_tracker/skeleton
    sub_openpose = nh->subscribe("frame", 1000, &CSkeletonProcessor::chatterCallback_fe, this); //Se suscribe al topic frame (esqueleto de la cámara ojo de pez)

    doc = fopen("/home/mapir-admin/Desktop/resultados.txt", "w+");
     
     esq_rgb[1].traslacion[0] = 0.5;
     esq_rgb[1].traslacion[1] = 0.0;
     esq_rgb[1].traslacion[2] = 0.0;
  }

  ~CSkeletonProcessor() {
    fclose(doc);
  }

  void fuseSkeletons()
  {
    
    // transform coordinates
    camaras_ref();

    int cont = 0;

    for(int i=0; i<= 4; i++)
    {

      // set of filters      
      if( !skeleton_ready[i] )
      {
        fprintf(doc,"0.000,0.000,0.000,");
        continue;
      }

      const double lapse = esq_rgb[i].timestamp - esq_rgb[0].timestamp;
      if( esq_rgb_ref[i].status == 0 || fabs(lapse) > MAX_LAPSE )
      {
        fprintf(doc,"0.000,0.000,0.000,");
        continue;
      }

      // valid skeleton
      esq_fused.posicion_cabeza =  esq_fused.posicion_cabeza + esq_rgb_ref[i].posicion_cabeza; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_cuello =  esq_fused.posicion_cuello + esq_rgb_ref[i].posicion_cuello; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_hombro =  esq_fused.posicion_hombro + esq_rgb_ref[i].posicion_hombro; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_columna_arriba =  esq_fused.posicion_columna_arriba + esq_rgb_ref[i].posicion_columna_arriba; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_columna_medio =  esq_fused.posicion_columna_medio + esq_rgb_ref[i].posicion_columna_medio; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_columna_abajo =  esq_fused.posicion_columna_abajo + esq_rgb_ref[i].posicion_columna_abajo; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_hombro_izquierdo =  esq_fused.posicion_hombro_izquierdo + esq_rgb_ref[i].posicion_hombro_izquierdo; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_codo_izquierdo =  esq_fused.posicion_codo_izquierdo + esq_rgb_ref[i].posicion_codo_izquierdo; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_mano_izquierda =  esq_fused.posicion_mano_izquierda + esq_rgb_ref[i].posicion_mano_izquierda; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_hombro_derecho =  esq_fused.posicion_hombro_derecho + esq_rgb_ref[i].posicion_hombro_derecho; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_codo_derecho =  esq_fused.posicion_codo_derecho + esq_rgb_ref[i].posicion_codo_derecho; //*(esq_rgb_ref[i].status*0.5);
      esq_fused.posicion_mano_derecha =  esq_fused.posicion_mano_derecha + esq_rgb_ref[i].posicion_mano_derecha; //*(esq_rgb_ref[i].status*0.5);
      cont++;

      skeleton_ready[i] = false;

      fprintf(doc, "%.3f,%.3f,%.3f,",esq_rgb_ref[i].posicion_cabeza[0],esq_rgb_ref[i].posicion_cabeza[1],esq_rgb_ref[i].posicion_cabeza[2]);      
    }

    esq_fused.posicion_cabeza = esq_fused.posicion_cabeza/cont;
    esq_fused.posicion_cuello = esq_fused.posicion_cuello/cont;
    esq_fused.posicion_hombro = esq_fused.posicion_hombro/cont;
    esq_fused.posicion_columna_arriba = esq_fused.posicion_columna_arriba/cont;
    esq_fused.posicion_columna_medio = esq_fused.posicion_columna_medio/cont;
    esq_fused.posicion_columna_abajo = esq_fused.posicion_columna_abajo/cont;
    esq_fused.posicion_hombro_izquierdo = esq_fused.posicion_hombro_izquierdo/cont;
    esq_fused.posicion_codo_izquierdo = esq_fused.posicion_codo_izquierdo/cont;
    esq_fused.posicion_mano_izquierda = esq_fused.posicion_mano_izquierda/cont;
    esq_fused.posicion_hombro_derecho = esq_fused.posicion_hombro_derecho/cont;
    esq_fused.posicion_codo_derecho = esq_fused.posicion_codo_derecho/cont;
    esq_fused.posicion_mano_derecha = esq_fused.posicion_mano_derecha/cont;

    // save results (DEBUG)
    fprintf(doc, "%.3f,%.3f,%.3f\n",esq_fused.posicion_cabeza[0],esq_fused.posicion_cabeza[1],esq_fused.posicion_cabeza[2]);
  }  

  // callbacks
  // this callback accepts the index of the camera to process
  void chatterCallback_rgb(const body_tracker_msgs::Skeleton::ConstPtr&  data, const int idx)
  {
    // DEBUG
    // ROS_INFO("join_position_head_x is: %f", data->joint_position_head.x);
    // ROS_INFO("join_position_head_y is: %f", data->joint_position_head.y);
    // ROS_INFO("join_position_head_z is: %f", data->joint_position_head.z);
    if( skeleton_ready[idx] == false )
    {
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

      // Guardar el tiempo en el que se guardan los datos
      esq_rgb[idx].timestamp = ros::Time::now().toSec();
      esq_rgb[idx].status = data->tracking_status;

      // activar el flag de que el esqueleto esta listo para ser procesado
      skeleton_ready[idx] = true;
    }
  }

  void chatterCallback_fe(const ros_openpose::Frame::ConstPtr&  data_fe)
  {
    // // DEBUG
    // ROS_INFO("position nose x is: %f", data_fe->persons[0].bodyParts[0].pixel.x);
    // ROS_INFO("position nose y is: %f", data_fe->persons[0].bodyParts[0].pixel.y);

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

  while(ros::ok()) {
    ros::spinOnce();
    //sp.fuseSkeletons();
  }

  return 0;
}