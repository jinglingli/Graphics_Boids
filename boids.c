#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "utility.h"
#include "boids.h"
#include "hw3.h"

Bird *make_bird(GLfloat x, GLfloat y, GLfloat z, GLfloat birth_angle, GLfloat birth_speed, GLfloat wing1, GLfloat wing2) {
    Bird *new = NULL;
    if ((new = (Bird *) malloc(sizeof(Bird))) !=NULL){
        new->old_loc = assign_values(x, y, z);
        new->new_loc = assign_values(x, y, z);
        new->dir_angle = ANGLE_OFFSET+birth_angle;
        new->birth_speed = birth_speed;
        new->wing_angles[0] = wing1; new->wing_angles[1] = wing2;
        new->wing_rate[0] = RATE_WING_ADD+rand()%RATE_WING_MOD; new->wing_rate[1] = RATE_WING_ADD+rand()%RATE_WING_MOD;
        new->velocity[0] = assign_values(birth_speed*cos(degree_to_radian(birth_angle)), birth_speed*sin(degree_to_radian(birth_angle)), 0);
        new->velocity[1] = assign_values(birth_speed*cos(degree_to_radian(birth_angle)), birth_speed*sin(degree_to_radian(birth_angle)), 0);
        new->obstacle_enter = FALSE;
        new->obstacle_escape = rand()%2;
    }
    num_malloc++;
    return new;
}

Boids *make_boids(Bird *bird) {
    Boids *new = NULL;
    if ((new = (Boids *) malloc(sizeof(Boids))) !=NULL){
        new->prev = NULL; new->next = NULL;
        new->cur_bird = bird;
    }
    num_malloc++;
    return new;
}

void init_boid_vertices() {
    cur_num_boids = 0;
    GLfloat boid_temp_vertices[][3]= {
        {BOID_RADIUS/2, 0, 0},
        {-BOID_RADIUS/2, 0, 0},
        {-BOID_RADIUS/2, -sin(degree_to_radian(ANGLE_WING))*2*BOID_RADIUS, cos(degree_to_radian(ANGLE_WING))*2*BOID_RADIUS},
        {-BOID_RADIUS/2, sin(degree_to_radian(ANGLE_WING))*2*BOID_RADIUS, cos(degree_to_radian(ANGLE_WING))*2*BOID_RADIUS}
    };
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 3; j++) {
            boid_vertices[i][j] = boid_temp_vertices[i][j];
        }
    }
}

void append_boids(Boids *boid) {
    if (flock_head == NULL) {
        flock_head = boid;
        flock_tail = boid;
    } else {
        boid->prev = flock_tail;
        flock_tail->next = boid;
        flock_tail = boid;
    }
    cur_num_boids++;
}

void delete_boids(){
    Boids *last = flock_tail;
    if(cur_num_boids == 1){
        flock_tail = flock_head = NULL;
    }else{
        flock_tail = flock_tail->prev;
        flock_tail->next = NULL;
    }
    free(last->cur_bird);
    free(last);
    cur_num_boids--;
    num_free+=2;
}

void clear_boids() {
    Boids *tmp, *tmp2;
    for (tmp = flock_head; tmp != NULL; tmp = tmp2) {
        tmp2 = tmp->next;
        free(tmp->cur_bird);
        free(tmp);
        cur_num_boids--;
        num_free+=2;
    }
    flock_head = flock_tail = NULL;
}

void clear_allocated_memory(){
    clear_boids();
    #if(DEBUG)
        printf("Number of malloc calls=%d, number of free calls=%d.\n", num_malloc, num_free);
    #endif
}

void print_boids_info() {
    Boids *temp = flock_head;
    while(temp!=NULL){
        Bird *b = temp->cur_bird;
        print_vector(b->new_loc, "Bird Position");
        print_vector(b->velocity[1], "Bird Velocity");
        printf("Bird Angle angle: %f\n", b->dir_angle);
        temp = temp->next;
    }
    printf("\n");
}