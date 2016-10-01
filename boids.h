#include <GL/glu.h>
#ifdef __APPLE__
#include <GLFW/glfw3.h>
#else
#include <GLFW/glfw3.h>
#endif

#ifndef UTILITY_HEADER
#include "utility.h"
#endif

#define BOIDS_HEADER 1

#define BOID_RADIUS 50
#define BOIDS_NUM 10
#define DEFAULT_VELOCITY 40
#define DEFAULT_BOID_SPEED 40
#define BOID_SPEED_RANGE 10
#define ANGLE_WING 45
#define ANGLE_BASE1 30
#define ANGLE_BASE2 45
#define RATE_WING_ADD 3
#define RATE_WING_MOD 7
#define FLAP_MIN_ANGLE 20
#define FLAP_MAX_ANGLE 160

#define FLOCK_RANGE 2000
#define ANGLE_OFFSET 0

#define LOCAL_SEARCH_RANGE 600
#define SEPERATION_RADIUS 400
#define COHESION_WEIGHT 0.1
#define SEPARATION_WEIGHT 0.5
#define ALIGN_WEIGHT 0.1
#define GOAL_ALIGN_WEIGHT 3
#define OBSTACLE_WEIGHT 50

typedef struct _bird {
    Vector old_loc, new_loc;
    GLfloat dir_angle, birth_speed;
    GLfloat wing_angles[2];
    Vector velocity[2];
    GLfloat wing_rate[2];
    GLboolean obstacle_enter, obstacle_escape;
} Bird;

typedef struct _boids {
    Bird *cur_bird;
    struct _boids *prev;
    struct _boids *next;
} Boids;

typedef struct _obs {
    Vector loc;
    GLfloat radius;
} Obstacle;

Bird *make_bird(GLfloat x, GLfloat y, GLfloat z, GLfloat birth_angle, GLfloat birth_speed, GLfloat wing1, GLfloat wing2);
Boids *make_boids(Bird *bird);
Obstacle *make_obstacle(GLfloat obs_x, GLfloat obs_y, GLfloat obs_z, GLfloat radius);

void append_boids (Boids *boid);
void delete_boids(); //always delete the last one
void clear_boids();
void init_boid_vertices();

void draw_wireframe_boid(Bird *bird, GLfloat x, GLfloat y, GLfloat z, GLfloat angle);
void draw_bird(Bird *bird, GLfloat x, GLfloat y, GLfloat z, GLfloat angle);
void draw_shadow_bird(Bird *bird, GLfloat x, GLfloat y, GLfloat z, GLfloat angle);

void update_centroid_dist();
void update_trailing_param();
void update_side_param();

void update_goal();
void update_boids2();
void update_boids_wings();
void update_boid_velocity(Bird *bird);
void update_boid_angle(Bird *bird);
void update_boid_location(Bird *bird);
void update_old_values(Bird *bird);

Vector update_alignment(Bird *bird);
Vector update_cohesion(Bird *bird);
Vector update_separation(Bird *bird);
Vector tendencyTo(Bird *bird);
Vector avoid_obstacle(Bird *bird);

void print_boids_info();

Boids *flock_head;
Boids *flock_tail;

GLuint cur_num_boids;