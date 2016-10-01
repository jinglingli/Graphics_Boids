#include <GL/glu.h>
#ifdef __APPLE__
#include <GLFW/glfw3.h>
#else
#include <GLFW/glfw3.h>
#endif

#ifndef BOIDS_HEADER
#include "boids.h"
#endif

#ifndef UTILITY_HEADER
#include "utility.h"
#endif

#define DEBUG 0
#define NEG -1.0f
#define TWO_PI 3.14159*2
#define TWO_PI_DEG 360
#define degree_to_radian(angle) (angle*TWO_PI/TWO_PI_DEG)
#define radian_to_degree(angle) (angle*TWO_PI_DEG/(TWO_PI))
#define CHECKER_SIZE 400
#define BOARD_SIZE 50
#define WIDTH 800
#define HEIGHT 800
#define VIEW_X_SIZE 20000
#define VIEW_Y_SIZE 20000
#define DEFAULT_RATIO (VIEW_X_SIZE/VIEW_Y_SIZE)

#define DEFAULT_ANGLE 60
#define DEFAULT_TRAIL_ANGLE 30
#define DEFAULT_SIDE_ANGLE 45

#define MAX_NUM_OBS 10
#define OBSTACLE_HEIGHT 10000

#define GENERAL_XY_RANGE 15000
#define GENERAL_Z_RANGE 2500
#define BASE_Z_RANGE 1500
#define CUBE_SIDE_LEN 45

#define DEFAULT_SPEED_GOAL 40
#define DEFAULT_ANGLE_GOAL 0
#define Z_CHANGE 40
#define TURN_ANGLE 15
#define SPEED_CHANGE 1.10f

#define INI_BOX_SIZE 50
#define INI_FLOCK_X 2400.0f
#define INI_FLOCK_Y 150.0f
#define INI_FLOCK_Z 1200.0f
#define DEFAULT_GOAL_X 100
#define DEFAULT_GOAL_Y 0
#define DEFAULT_GOAL_Z 4000
#define DEFAULT_VIEW_X 10
#define DEFAULT_VIEW_Y 0
#define DEFAULT_VIEW_Z 10000
#define DEFAULT_OBJECT_X 0
#define DEFAULT_OBJECT_Y 0
#define DEFAULT_OBJECT_Z 100

#define PERSPEC_FAR_RATIO 3.0f

#define ZOOM_TOWER_CONSTANT 3
#define ZOOM_S_T_CONSTANT 0.1
#define MAX_VIEW_ANGLE 90
#define MIN_VIEW_ANGLE 30
#define MAX_T_VIEW_COEF 1
#define MIN_T_VIEW_COEF 0
#define MAX_S_VIEW_COEF 2
#define MIN_S_VIEW_COEF -2

#define max(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a > _b ? _a : _b; })

enum{WHITE, BLACK, GREEN, PINK, SKY_BLUE, PURPLE, LIGHT_GREY, BIRD_SHADOW, DARK_GREY, DEBUG_RED};
enum{FALSE, TRUE};

void init();
void init_goal();
void init_view();
void init_board();
void init_obstacles();

void resize(int width, int height);
void reshape(GLFWwindow *w, int width, int height);
void framebuffer_size_callback(GLFWwindow *w, int width, int height);
void keyboard(GLFWwindow *w, int key, int scancode, int action, int mods);

void create_initial_boids();
void create_vertex_order(int boardsize);
void create_vertex_position(int boardsize, int cell_side_length);
void create_vertex_color(int boardsize);
void create_one_boid(GLfloat birth_x, GLfloat birth_y, GLfloat birth_z);

void draw_goal();
void draw_boids();
void draw_checker();
void draw_obstacle();
void draw_one_obstacle(Obstacle obs);

void clear_allocated_memory();
void update_view();

void gluLookAt_matrix(GLfloat eyeX, GLfloat eyeY, GLfloat eyeZ, GLfloat centerX, GLfloat centerY, GLfloat centerZ, GLfloat upX, GLfloat upY, GLfloat upZ);
void gluPerspective_self(GLfloat fovy, GLfloat aspect, GLfloat zNear, GLfloat zFar);
void gluLookAt_self(GLfloat eyeX, GLfloat eyeY, GLfloat eyeZ, GLfloat centerX, GLfloat centerY, GLfloat centerZ, GLfloat upX, GLfloat upY, GLfloat upZ);

int num_malloc, num_free;
Vector goal_loc;
GLfloat goal_speed, goal_angle;
GLuint num_obstacles;

GLboolean pause;
GLfloat view_angle;
GLfloat t_view_coef, s_view_coef; //zoom in and zoom out rate for trailing view and side view
Vector centroid; //centroid of the flock
GLfloat Camera_x, Camera_y, Camera_z, Object_x, Object_y, Object_z, max_bird_dist, goal_c_dist; //goal_c_dist is d; max_bird_dist is r
GLbyte view_mode, up_x, up_y, up_z; //up vector for Glulookat
GLfloat t_view_x, t_view_y, t_view_z, s_view_x, s_view_y, s_view_z; //trailing view and side view parameters

Vector u_cg, M, norm_u; //vectors used to help calculating trailing view and side view parameters
Vector up, F, s, u; //vectors used to implement GluLookAt and GluPerspective

GLfloat boid_vertices[4][3];
GLfloat M_glulookat[16]; //transformation matrix
GLfloat M_perspective[16]; //transformation matrix

GLfloat goal_colors[8][3];
Obstacle Obstacles[MAX_NUM_OBS]; //set up at most 10 obstacles

GLuint checker_indices[BOARD_SIZE*BOARD_SIZE*4];
GLfloat checker_vertices[(BOARD_SIZE+1)*(BOARD_SIZE+1)][3];
GLfloat checker_colors[(BOARD_SIZE+1)*(BOARD_SIZE+1)][3];
