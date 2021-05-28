#pragma once
#include <Eigen/Dense>  // Matrices, arrays, transformadas, etc
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
  int status;
  double timestamp;

  // transformacion 3D entre este esqueleto y el de referencia (se rellena al inicio)
  // Matrix4f transformacion;
  // Matrix3f rotacion = MatrixXf::Identity(3, 3); // eye
  // Vector3f traslacion = MatrixXf::Zero(1,3); // zero vector
  Matrix3f rotacion; // eye
  Vector3f traslacion; // zero vector

  TSkeleton() {
    rotacion = Matrix3f::Identity(); // eye
    traslacion = Vector3f::Zero(); // zero vector

    status = 0;
    timestamp = 0.0;

    posicion_cabeza = Vector3f::Zero(); // zero vector
    posicion_cuello = Vector3f::Zero(); // zero vector
    posicion_hombro = Vector3f::Zero(); // zero vector
    posicion_columna_arriba = Vector3f::Zero(); // zero vector
    posicion_columna_medio = Vector3f::Zero(); // zero vector
    posicion_columna_abajo = Vector3f::Zero(); // zero vector

    posicion_hombro_izquierdo = Vector3f::Zero(); // zero vector
    posicion_codo_izquierdo = Vector3f::Zero(); // zero vector
    posicion_mano_izquierda = Vector3f::Zero(); // zero vector

    posicion_hombro_derecho = Vector3f::Zero(); // zero vector
    posicion_codo_derecho = Vector3f::Zero(); // zero vector
    posicion_mano_derecha = Vector3f::Zero(); // zero vector
  }
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

  TSkeleton_fe() {
    posicion_cabeza = Vector3f::Zero();
    posicion_cuello = Vector3f::Zero();
    posicion_cadera = Vector3f::Zero();

    posicion_ojo_izquierdo = Vector3f::Zero();
    posicion_oreja_izquierda = Vector3f::Zero();

    posicion_ojo_derecho = Vector3f::Zero();
    posicion_oreja_derecha = Vector3f::Zero();

    posicion_hombro_izquierdo = Vector3f::Zero();
    posicion_codo_izquierdo = Vector3f::Zero();
    posicion_mano_izquierda = Vector3f::Zero();
    posicion_cadera_izquierda = Vector3f::Zero();
    posicion_rodilla_izquierda = Vector3f::Zero();
    posicion_pie_izquierdo_talon = Vector3f::Zero();
    posicion_pie_izquierdo_punta = Vector3f::Zero();

    posicion_hombro_derecho = Vector3f::Zero();
    posicion_codo_derecho = Vector3f::Zero();
    posicion_mano_derecha = Vector3f::Zero();
    posicion_cadera_derecha = Vector3f::Zero();
    posicion_rodilla_derecha = Vector3f::Zero();
    posicion_pie_derecha_talon = Vector3f::Zero();
    posicion_pie_derecha_punta = Vector3f::Zero();
  }

};
