#include <stdio.h>
#include <stdlib.h>
#include "boids.h"
#include "hw3.h"

const Vector zero = {{0.0, 0.0, 0.0}};

void update_boids2() {
    #if (DEBUG)
        print_boids_info();
    #endif
    Boids *temp = flock_head;
    while(temp!=NULL){
        Bird *bird = temp->cur_bird;
        update_boid_velocity(bird);
        update_boid_location(bird);
        update_boid_angle(bird);
        temp = temp->next;
    }
    Boids *temp2 = flock_head;
    while(temp2!=NULL){
        Bird *bird = temp2->cur_bird;
        update_old_values(bird);
        temp2 = temp2->next;
    }
}

void update_boid_location(Bird *bird) {
    bird->new_loc = add_vec_struct(bird->old_loc, bird->velocity[1]);
}

void update_old_values(Bird *bird) {
    bird->old_loc = add_vec_struct(bird->new_loc, zero);
    bird->velocity[0] = add_vec_struct(bird->velocity[1], zero);
}

Vector avoid_obstacle(Bird *bird){
    Vector tendency = assign_values(0,0,0);
    Vector temp = assign_values(0,0,0);
    for(int i=0; i<num_obstacles; i++){
        GLfloat dist = distance3d((Obstacles[i].loc).xyz[0], (Obstacles[i].loc).xyz[1], 0, (bird->old_loc).xyz[0], (bird->old_loc).xyz[1], 0);
        if(dist < Obstacles[i].radius){
            temp = add_vec_struct(Obstacles[i].loc, scalar_multi_vec_struct(bird->old_loc, NEG));
            if(bird->obstacle_escape){
                tendency = vector_product_struct(temp.xyz[0], temp.xyz[1], temp.xyz[2], 0, 0, 1); //turn right
            }else{
                tendency = vector_product_struct(0, 0, 1, temp.xyz[0], temp.xyz[1], temp.xyz[2]); //turn left
            }
            tendency = normalize_vec_struct(tendency);
            bird->obstacle_enter = TRUE;
            return tendency;
        }else if(bird->obstacle_enter && dist < Obstacles[i].radius*2){
            tendency = add_vec_struct(bird->velocity[0], zero);
            return tendency;
        }else{
            for(int j=0; j<3; j++){
                tendency.xyz[i] = 0;
            }
            bird->obstacle_enter = FALSE;
        }
    }
    return tendency;
}

void update_boid_angle(Bird *bird) {
    GLfloat angle = calculate_angle(1, 0, (bird->velocity[1]).xyz[0], (bird->velocity[1]).xyz[1]);
    bird->dir_angle = angle;
}

void update_boid_velocity(Bird *bird) {
    Vector alignment = update_alignment(bird);
    Vector cohesion = update_cohesion(bird);
    Vector separation = update_separation(bird);
    Vector tendency = tendencyTo(bird);
    Vector avoidance = avoid_obstacle(bird);
    
    alignment = scalar_multi_vec_struct(alignment, ALIGN_WEIGHT);
    cohesion = scalar_multi_vec_struct(cohesion, COHESION_WEIGHT);
    separation = scalar_multi_vec_struct(separation, SEPARATION_WEIGHT);
    tendency = scalar_multi_vec_struct(tendency, GOAL_ALIGN_WEIGHT);
    avoidance = scalar_multi_vec_struct(avoidance, OBSTACLE_WEIGHT);
    
    Vector v1 = add_vec_struct(alignment, cohesion);
    Vector v2 = add_vec_struct(separation, tendency);
    Vector all_effects = add_vec_struct(v1, v2);
    all_effects = add_vec_struct(avoidance, all_effects);
    
    bird->velocity[1] = add_vec_struct(bird->velocity[1], all_effects);
    bird->velocity[1] = normalize_vec_struct(bird->velocity[1]);
    bird->velocity[1] = scalar_multi_vec_struct(bird->velocity[1], DEFAULT_BOID_SPEED);
    
    #if (DEBUG)
        print_vector(alignment, "alignment");
        print_vector(cohesion, "cohesion");
        print_vector(separation, "separation");
        print_vector(tendency, "tendency");
        print_vector(avoidance, "avoidance");
        print_vector(all_effects, "all_effects");
        print_vector(bird->velocity[1], "bird->velocity[1]");
    #endif
}

Vector update_alignment(Bird *bird) {
    Vector vector= assign_values(0, 0, 0);
    GLuint neighborCount = 0;
    Boids *temp = flock_head;
    
    while(temp!=NULL){
        Bird *b = temp->cur_bird;
        GLfloat dist = distance3dv(bird->old_loc, b->old_loc);
        if(dist < LOCAL_SEARCH_RANGE && dist>0) {
            vector = add_vec_struct(vector, b->velocity[0]);
            neighborCount++;
        }
        temp = temp->next;
    }
    if(neighborCount == 0) {
        return vector;
    } else {
        vector = scalar_multi_vec_struct(vector, (1/neighborCount));
        vector = add_vec_struct(vector, scalar_multi_vec_struct(bird->velocity[0], NEG));
        vector = normalize_vec_struct(vector);
        return vector;
    }
}

Vector update_cohesion(Bird *bird) {
    Vector vector= assign_values(0, 0, 0);
    GLuint neighborCount = 0;
    Boids *temp = flock_head;
    
    while(temp!=NULL){
        Bird *b = temp->cur_bird;
        GLfloat dist = distance3dv(bird->old_loc, b->old_loc);
        if(dist < LOCAL_SEARCH_RANGE && dist>0) {
            vector = add_vec_struct(vector, b->old_loc);
            neighborCount++;
        }
        temp = temp->next;
    }
    if(neighborCount == 0) {
        return vector;
    } else {
        vector = scalar_multi_vec_struct(vector, (1/neighborCount));
        vector = add_vec_struct(vector, scalar_multi_vec_struct(bird->old_loc, NEG));
        vector = normalize_vec_struct(vector);
        return vector;
    }
}

Vector update_separation(Bird *bird) {
    Vector vector= assign_values(0, 0, 0);
    GLuint neighborCount = 0;
    Boids *temp = flock_head;
    while(temp!=NULL){
        Bird *b = temp->cur_bird;
        GLfloat dist = distance3dv(bird->old_loc, b->old_loc);
        if(dist < SEPERATION_RADIUS && dist>0){
            vector = add_vec_struct(vector, bird->old_loc);
            vector = add_vec_struct(vector, scalar_multi_vec_struct(b->old_loc, NEG));
            neighborCount++;
        }
        temp = temp->next;
    }
    if(neighborCount == 0) {
        return vector;
    } else {
        vector = normalize_vec_struct(vector);
        return vector;
    }
}

Vector tendencyTo(Bird *bird) {
    Vector tendency;
    tendency = add_vec_struct(bird->old_loc, zero);
    tendency = add_vec_struct(goal_loc, scalar_multi_vec_struct(tendency, NEG));
    tendency = normalize_vec_struct(tendency);
    return tendency;
}

void update_centroid_dist(){
    Boids *temp = flock_head;
    Vector center = assign_values(0, 0, 0);
    while(temp!=NULL){
        Bird *bird = temp->cur_bird;
        GLfloat dist_check = distance3dv(bird->new_loc, centroid);
        if(dist_check){
            center = add_vec_struct(center, bird->new_loc);
        }
        temp = temp->next;
    }
    centroid = scalar_multi_vec_struct(center, (1.0/cur_num_boids));
    goal_c_dist = distance3dv(goal_loc, centroid);
    
    temp = flock_head;
    max_bird_dist = 0;
    GLfloat dist = 0;
    while(temp!=NULL){
        Bird *bird = temp->cur_bird;
        dist = distance3dv(bird->new_loc, centroid);
        if(dist > max_bird_dist && dist <= FLOCK_RANGE){
            max_bird_dist = dist;
        }
        temp = temp->next;
    }
    u_cg = add_vec_struct(goal_loc, scalar_multi_vec_struct(centroid, NEG));
    M = scalar_multi_vec_struct(add_vec_struct(centroid, goal_loc), (1.0/2));
}

void update_trailing_param(){
    norm_u = scalar_multi_vec_struct(u_cg, NEG);
    Vector p = normalize_vec_struct(norm_u);
    GLfloat d_5r = goal_c_dist+5*max_bird_dist;
    GLfloat d_r = goal_c_dist+1*max_bird_dist;
    t_view_x = centroid.xyz[0] + t_view_coef * d_5r * (p.xyz[0]);
    t_view_y = centroid.xyz[1] + t_view_coef * d_5r * (p.xyz[1]);
    t_view_z = centroid.xyz[2] + t_view_coef * (d_5r * (p.xyz[2]) + d_r);
}

void update_side_param(){
    Vector p = vector_product_struct(u_cg.xyz[0], u_cg.xyz[1], u_cg.xyz[2], 0, 0, 1);
    GLfloat d_2r = goal_c_dist+2*max_bird_dist;
    GLfloat d_r = goal_c_dist+1*max_bird_dist;
    p = normalize_vec_struct(p);
    s_view_x = centroid.xyz[0] + s_view_coef * d_2r * (p.xyz[0]);
    s_view_y = centroid.xyz[1] + s_view_coef *  d_2r * (p.xyz[1]);
    s_view_z = centroid.xyz[2] + s_view_coef *  d_2r * (p.xyz[2]) + d_r;
}

void update_boids_wings() {
    Boids *temp = flock_head;
    while(temp!=NULL){
        if(temp->cur_bird->wing_angles[0]<= FLAP_MIN_ANGLE || temp->cur_bird->wing_angles[0] >= FLAP_MAX_ANGLE)
            temp->cur_bird->wing_rate[0] = - temp->cur_bird->wing_rate[0];
        if(temp->cur_bird->wing_angles[1]<= FLAP_MIN_ANGLE || temp->cur_bird->wing_angles[1] >= FLAP_MAX_ANGLE)
            temp->cur_bird->wing_rate[1] = - temp->cur_bird->wing_rate[1];
        
        temp->cur_bird->wing_angles[0] += temp->cur_bird->wing_rate[0];
        temp->cur_bird->wing_angles[1] += temp->cur_bird->wing_rate[1];
        temp = temp->next;
    }
}