#include <GL/glu.h>
#ifdef __APPLE__
#include <GLFW/glfw3.h>
#else
#include <GLFW/glfw3.h>
#endif
#define UTILITY_HEADER 1

typedef struct _vector {
    GLfloat xyz[3];
} Vector;

Vector normalize_vec_struct(Vector vec);
Vector add_vec_struct(Vector vec1, Vector vec2);
Vector scalar_multi_vec_struct(Vector vec1, GLfloat scalar);
Vector vector_product_struct(GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2);
GLfloat distance3dv(Vector v1, Vector v2);
GLfloat distance3d(GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2);
GLfloat calculate_angle(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2);
Vector assign_values(GLfloat x, GLfloat y, GLfloat z);

void print_vector(Vector vec, char* str);

