#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "utility.h"
#include "hw3.h"

Vector assign_values(GLfloat x, GLfloat y, GLfloat z){
    Vector v;
    v.xyz[0] = x;
    v.xyz[1] = y;
    v.xyz[2] = z;
    return v;
}

Vector normalize_vec_struct(Vector vec) {
    GLfloat dist = distance3d(vec.xyz[0], vec.xyz[1], vec.xyz[2], 0, 0, 0);
    vec.xyz[0] = vec.xyz[0]/dist;
    vec.xyz[1] = vec.xyz[1]/dist;
    vec.xyz[2] = vec.xyz[2]/dist;
    return vec;
}

Vector add_vec_struct(Vector vec1, Vector vec2){
    vec1.xyz[0] += vec2.xyz[0];
    vec1.xyz[1] += vec2.xyz[1];
    vec1.xyz[2] += vec2.xyz[2];
    return vec1;
}

Vector scalar_multi_vec_struct(Vector vec1, GLfloat scalar){
    vec1.xyz[0] *= scalar;
    vec1.xyz[1] *= scalar;
    vec1.xyz[2] *= scalar;
    return vec1;
}

Vector vector_product_struct(GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2){
    Vector p;
    p.xyz[0] = y1*z2 - z1*y2;
    p.xyz[1] = z1*x2 - x1*z2;
    p.xyz[2] = x1*y2 - y1*x2;
    return p;
}

GLfloat calculate_angle(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2) {
    GLfloat dot = x1*x2 + y1*y2;      //dot product
    GLfloat det = x1*y2 - y1*x2;      // determinant
    GLfloat angle = atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)
    return radian_to_degree(angle); //return degree
}

GLfloat distance3d(GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2){
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
}

GLfloat distance3dv(Vector v1, Vector v2){
    return distance3d(v1.xyz[0], v1.xyz[1], v1.xyz[2], v2.xyz[0], v2.xyz[1], v2.xyz[2]);
}

void print_vector(Vector vec, char* str){
    printf("Vec %s x: %f, y: %f, z: %f\n", str, vec.xyz[0], vec.xyz[1], vec.xyz[2]);
}