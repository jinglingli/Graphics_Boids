#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "utility.h"
#include "boids.h"
#include "hw3.h"

GLfloat glc[][3] = {
    {1.0, 1.0, 1.0},
    {0.0, 0.0, 0.0},
    {0.604, 0.804, 0.196},
    {0.859, 0.439, 0.576},
    {0, 0.807, 0.819},
    {0.482, 0.408, 0.933},
    {0.5, 0.5, 0.5},
    {0.4 ,0.4, 0.4},
    {0.9 ,0.9, 0.9},
    {1.0, 0.0, 0.0}
};

GLfloat goal_vertices[][3] = {
    {-CUBE_SIDE_LEN,-CUBE_SIDE_LEN,CUBE_SIDE_LEN},
    {-CUBE_SIDE_LEN,CUBE_SIDE_LEN,CUBE_SIDE_LEN},
    {CUBE_SIDE_LEN,CUBE_SIDE_LEN,CUBE_SIDE_LEN},
    {CUBE_SIDE_LEN,-CUBE_SIDE_LEN,CUBE_SIDE_LEN},
    {-CUBE_SIDE_LEN,-CUBE_SIDE_LEN,-CUBE_SIDE_LEN},
    {-CUBE_SIDE_LEN,CUBE_SIDE_LEN,-CUBE_SIDE_LEN},
    {CUBE_SIDE_LEN,CUBE_SIDE_LEN,-CUBE_SIDE_LEN},
    {CUBE_SIDE_LEN,-CUBE_SIDE_LEN,-CUBE_SIDE_LEN}
};

GLfloat obstacle_vertices[][3] = {
    {-CUBE_SIDE_LEN,-CUBE_SIDE_LEN,CUBE_SIDE_LEN},
    {-CUBE_SIDE_LEN,CUBE_SIDE_LEN,OBSTACLE_HEIGHT*CUBE_SIDE_LEN},
    {CUBE_SIDE_LEN,CUBE_SIDE_LEN,OBSTACLE_HEIGHT*CUBE_SIDE_LEN},
    {CUBE_SIDE_LEN,-CUBE_SIDE_LEN,CUBE_SIDE_LEN},
    {-CUBE_SIDE_LEN,-CUBE_SIDE_LEN,-CUBE_SIDE_LEN},
    {-CUBE_SIDE_LEN,CUBE_SIDE_LEN,-OBSTACLE_HEIGHT},
    {CUBE_SIDE_LEN,CUBE_SIDE_LEN,-OBSTACLE_HEIGHT},
    {CUBE_SIDE_LEN,-CUBE_SIDE_LEN,-CUBE_SIDE_LEN}
};

GLubyte goal_indices[24] = {
    0,3,2,1,
    2,3,7,6,
    0,4,7,3,
    1,2,6,5,
    4,5,6,7,
    0,1,5,4
};

GLfloat boid_colors[][3] = {
    {1.0, 1.0, 1.0},
    {1.0, 1.0, 1.0},
    {0.0, 1.0, 0.5},
    {0.0, 0.5, 1.0}
};

GLubyte boid_indices[6] = {
    0, 1, 2,
    1, 0, 3
};

GLubyte boid_wireframe_indices[12] = {
    0, 1,
    1, 2,
    2, 0,
    1, 0,
    0, 3,
    3, 1
};

void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    srand(time(NULL));
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective_self(DEFAULT_ANGLE, DEFAULT_RATIO, 0.5, PERSPEC_FAR_RATIO*VIEW_X_SIZE*cos(degree_to_radian(DEFAULT_ANGLE)));
    
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_FLAT);
    
    num_malloc = 0; num_free = 0; pause = FALSE;
    init_board();
    init_boid_vertices();
    create_initial_boids();
    init_goal();
    init_view();
    update_view();
    init_obstacles();
}

void init_board(){
    create_vertex_order(BOARD_SIZE);
    create_vertex_position(BOARD_SIZE, CHECKER_SIZE);
    create_vertex_color(BOARD_SIZE);
}

void init_view(){
    up_x = 0;
    up_y = 0;
    up_z = 1;
    view_mode = 'f';
    view_angle = DEFAULT_ANGLE;
    centroid = assign_values(INI_FLOCK_X, INI_FLOCK_Y, INI_FLOCK_Z);
    Camera_x = DEFAULT_VIEW_X;
    Camera_y = DEFAULT_VIEW_Y;
    Camera_z = DEFAULT_VIEW_Z;
    Object_x = DEFAULT_OBJECT_X;
    Object_y = DEFAULT_OBJECT_Y;
    Object_z = DEFAULT_OBJECT_Z;
    norm_u = assign_values(1, 0, 0);
    up = assign_values(0, 0, 0);
    F = assign_values(0, 0, 0);
    s = assign_values(0, 0, 0);
    u  = assign_values(0, 0, 0);
    t_view_coef = 1;
    s_view_coef = 1;
}

void init_obstacles(){
    FILE *myFile; char c;
    myFile = fopen("obstacles_position.txt", "r");
    if(myFile==NULL){
        printf("NULL POINTER \n");
        num_obstacles = 0;
        return;
    }
    c = fgetc(myFile);
    c = c - '0';
    num_obstacles = c;
    for(int i = 0; i < c; i++) {
        fscanf(myFile, "%f,%f,%f,%f", &((Obstacles[i].loc).xyz[0]), &((Obstacles[i].loc).xyz[1]), &((Obstacles[i].loc).xyz[2]), &(Obstacles[i].radius));
    }
    fclose(myFile);
}

int main(int argc, char **argv) {
    GLFWwindow* window;
    if (!glfwInit())
        exit(EXIT_FAILURE);
    window = glfwCreateWindow(WIDTH, HEIGHT, "Graphics HW3", NULL, NULL);
    if (!window){
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwSetWindowPos(window, 100, 100);
    glfwMakeContextCurrent(window);
    glfwSetWindowSizeCallback(window, reshape);
    glfwSetKeyCallback(window, keyboard);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    
    init();
    
    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearDepth(1.0);
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective_self(view_angle, DEFAULT_RATIO, 0.5, PERSPEC_FAR_RATIO*VIEW_X_SIZE*cos(degree_to_radian(DEFAULT_ANGLE)));
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt_self( Camera_x, Camera_y, Camera_z, Object_x, Object_y, Object_z, up_x, up_y, up_z);
   
        draw_goal();
        draw_checker();
        if(!pause){
            update_boids2();
            update_boids_wings();
        }
        draw_boids();
        draw_obstacle();
        update_view();
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    glfwTerminate();
    exit(EXIT_SUCCESS);
}

void update_view(){
    update_centroid_dist();
    update_trailing_param();
    update_side_param();
    switch (view_mode) {
        case 'f':
            Camera_x = DEFAULT_VIEW_X;
            Camera_y = DEFAULT_VIEW_Y;
            Camera_z = DEFAULT_VIEW_Z;
            break;
        case 't':
            Camera_x = t_view_x;
            Camera_y = t_view_y;
            Camera_z = t_view_z;
            view_angle = DEFAULT_TRAIL_ANGLE;
            break;
        case 's':
            update_side_param();
            Camera_x = s_view_x;
            Camera_y = s_view_y;
            Camera_z = s_view_z;
            view_angle = DEFAULT_SIDE_ANGLE;
            break;
        default:
            break;
    }
    
    Object_x = M.xyz[0];
    Object_y = M.xyz[1];
    Object_z = M.xyz[2];
    /**
     If there is no boid, we will look at the goal using default view
     **/
    if(cur_num_boids == 0){
        Camera_x = DEFAULT_VIEW_X;
        Camera_y = DEFAULT_VIEW_Y;
        Camera_z = DEFAULT_VIEW_Z;
        Object_x = goal_loc.xyz[0];
        Object_y = goal_loc.xyz[1];
        Object_z = goal_loc.xyz[2];
    }
}

void keyboard(GLFWwindow *w, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS){
        switch(key) {
            case GLFW_KEY_ESCAPE:
            case 'q' :
            case 'Q':
                glfwSetWindowShouldClose(w, GL_TRUE);
                clear_allocated_memory();
                break;
            case 61 :{
                GLfloat birth_x = -GENERAL_XY_RANGE/2 + rand()%GENERAL_XY_RANGE;
                GLfloat birth_y = -GENERAL_XY_RANGE/2 + rand()%GENERAL_XY_RANGE;
                GLfloat birth_z = rand()%GENERAL_Z_RANGE + BASE_Z_RANGE;
                create_one_boid(birth_x, birth_y, birth_z);
                break;
            }
            case 45:
                if(cur_num_boids>0){
                    delete_boids();
                }
                break;
            case 'p':
            case 'P':
                pause = !pause;
                break;
            case 'f':
            case 'F': {
                view_mode = 'f';
                view_angle = DEFAULT_ANGLE;
                break;
            }
            case 't':
            case 'T': {
                view_mode = 't';
                t_view_coef = 1;
                break;
            }
            case 's':
            case 'S': {
                view_mode = 's';
                s_view_coef = 1;
                break;
            }
            default:
                break;
        }
    }
    if ((action == GLFW_PRESS)||(action == GLFW_REPEAT)) {
        switch(key) {
            case 'd':
            case 'D':
                if(pause){
                    update_goal();
                    update_boids2();
                    update_boids_wings();
                    update_view();
                }
                break;
            case 'z':
            case 'Z':{
                switch(view_mode) {
                    case 's':
                        if(s_view_coef > MIN_S_VIEW_COEF)
                            s_view_coef -= ZOOM_S_T_CONSTANT;
                        break;
                    case 't':
                        if(t_view_coef > MIN_T_VIEW_COEF)
                            t_view_coef -= ZOOM_S_T_CONSTANT;
                        break;
                    case 'f':
                        if(view_angle > MIN_VIEW_ANGLE)
                            view_angle -= ZOOM_TOWER_CONSTANT;
                        break;
                        
                    default:
                        break;
                }
                break;
            }
            case 'x':
            case 'X':{
                switch(view_mode) {
                    case 's':
                        if(s_view_coef < MAX_S_VIEW_COEF)
                            s_view_coef += ZOOM_S_T_CONSTANT;
                        break;
                    case 't':
                        if(t_view_coef < MAX_T_VIEW_COEF)
                            t_view_coef += ZOOM_S_T_CONSTANT;
                        
                        break;
                    case 'f':
                        if(view_angle < MAX_VIEW_ANGLE)
                            view_angle += ZOOM_TOWER_CONSTANT;
                        
                        break;
                    default:
                        break;
                }
                break;
            }
            case GLFW_KEY_LEFT:
                goal_angle += TURN_ANGLE;
                break;
            case GLFW_KEY_RIGHT:
                goal_angle -= TURN_ANGLE;
                break;
            case GLFW_KEY_UP:
                goal_loc.xyz[2] += Z_CHANGE;
                break;
            case GLFW_KEY_DOWN:
                if(goal_loc.xyz[2] > Z_CHANGE)
                    goal_loc.xyz[2] -= Z_CHANGE;
                break;
            case 'b':
            case 'B': //speeding up
                goal_speed *= SPEED_CHANGE;
                break;
            case 'v':
            case 'V': //slowing down
                goal_speed /= SPEED_CHANGE;
                break;
            default:
                break;
        }
    }
}


void draw_bird(Bird *bird, GLfloat x, GLfloat y, GLfloat z, GLfloat angle) {
    boid_vertices[2][1] = -sin(degree_to_radian(bird->wing_angles[0]))*2*BOID_RADIUS;
    boid_vertices[2][2] = cos(degree_to_radian(bird->wing_angles[0]))*2*BOID_RADIUS;
    boid_vertices[3][1] = sin(degree_to_radian(bird->wing_angles[1]))*2*BOID_RADIUS;
    boid_vertices[3][2] = cos(degree_to_radian(bird->wing_angles[1]))*2*BOID_RADIUS;
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    
    glVertexPointer(3, GL_FLOAT, 0, boid_vertices);
    glColorPointer(3, GL_FLOAT, 0, boid_colors);
    
    glTranslatef(x, y, z);
    glRotatef(angle, 0, 0, 1);
    
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, boid_indices);
    
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    
    glPopMatrix();
}

void draw_wireframe_boid(Bird *bird, GLfloat x, GLfloat y, GLfloat z, GLfloat angle) {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    glColor3fv(glc[WHITE]);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, boid_vertices);
    
    glTranslatef(x, y, z);
    glRotatef(angle, 0, 0, 1);
    
    glDrawElements(GL_LINES, 12, GL_UNSIGNED_BYTE, boid_wireframe_indices);
    glPopMatrix();
}

void draw_boids(){
    Boids *temp = flock_head;
    while(temp!=NULL){
        Bird *bird = temp->cur_bird;
        GLfloat bird_x = (bird->new_loc).xyz[0];
        GLfloat bird_y = (bird->new_loc).xyz[1];
        GLfloat bird_z = (bird->new_loc).xyz[2];
        GLfloat bird_angle = bird->dir_angle;
        draw_bird(bird, bird_x, bird_y, bird_z, bird_angle);
        draw_shadow_bird(bird, bird_x, bird_y, bird_z, bird_angle);
        draw_wireframe_boid(bird, bird_x, bird_y, bird_z, bird_angle);
        temp = temp->next;
    }
    
}

void init_goal() {
    for(int i = 0; i<8; i++){
        for(int j = 0; j<3; j++)
            goal_colors[i][j] = glc[i][j];
    }
    goal_loc = assign_values(DEFAULT_GOAL_X, DEFAULT_GOAL_Y, DEFAULT_GOAL_Z);
    goal_speed = DEFAULT_SPEED_GOAL;
    goal_angle = DEFAULT_ANGLE_GOAL;
}

void update_goal(){
    if(goal_loc.xyz[0] > VIEW_X_SIZE/2 || goal_loc.xyz[0] < -VIEW_X_SIZE/2 || goal_loc.xyz[1] > VIEW_X_SIZE/2 || goal_loc.xyz[1] < -VIEW_X_SIZE/2){
        goal_angle += 180;
    }
    goal_loc.xyz[0] = goal_loc.xyz[0] + goal_speed*cos(degree_to_radian(goal_angle));
    goal_loc.xyz[1] = goal_loc.xyz[1] + goal_speed*sin(degree_to_radian(goal_angle));
}

void draw_goal() {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    glShadeModel(GL_SMOOTH);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, goal_vertices);
    glColorPointer(3, GL_FLOAT, 0, goal_colors);
    
    if(!pause){
        update_goal();
    }
    
    glTranslatef(goal_loc.xyz[0], goal_loc.xyz[1], goal_loc.xyz[2]);
    glRotatef(goal_angle, 0,0,1);
    
    glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, goal_indices);
    
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    
    glShadeModel(GL_FLAT);
    glPopMatrix();
}
void create_initial_boids(){
    for(int i = 0; i < BOIDS_NUM; i++) {
        GLfloat birth_x = -INI_BOX_SIZE/2 + rand()%INI_BOX_SIZE + INI_FLOCK_X;
        GLfloat birth_y = -INI_BOX_SIZE/2 + rand()%INI_BOX_SIZE + INI_FLOCK_Y;
        GLfloat birth_z = -INI_BOX_SIZE/2 + rand()%INI_BOX_SIZE + INI_FLOCK_Z;
        create_one_boid(birth_x, birth_y, birth_z);
    }
}

void create_one_boid(GLfloat birth_x, GLfloat birth_y, GLfloat birth_z){
    GLfloat birth_angle = rand()%TWO_PI_DEG;
    GLfloat birth_speed = DEFAULT_BOID_SPEED-5 + rand()%BOID_SPEED_RANGE;
    GLfloat wing_angle1 = rand()%ANGLE_BASE1 + ANGLE_WING;
    GLfloat wing_angle2 = rand()%ANGLE_BASE2 + ANGLE_WING;
    Bird *bird = make_bird(birth_x, birth_y, birth_z, birth_angle, birth_speed, wing_angle1, wing_angle2);
    Boids *boid = make_boids(bird);
    append_boids(boid);
}

void resize(int width, int height) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective_self(view_angle, DEFAULT_RATIO, 0.5, 2.0*VIEW_X_SIZE*cos(degree_to_radian(DEFAULT_ANGLE)));
    glMatrixMode(GL_MODELVIEW);
    
}
void reshape(GLFWwindow *w, int width, int height) {
    resize(width, height);
}
void framebuffer_size_callback(GLFWwindow *w, int width, int height) {
    glViewport(0, 0, width, height);
}

void create_vertex_order(int boardsize) {
    int total_cells_num = (boardsize+1)*(boardsize+1);
    int i, j=0;
    
    for(i=0; i<total_cells_num; i++){
        if(((i-boardsize)%(boardsize+1)) != 0){
            checker_indices[j] = i;
            checker_indices[j+1] = i+1+boardsize;
            checker_indices[j+2] = i+2+boardsize;
            checker_indices[j+3] = i+1;
            j+=4;
        }
    }
}

void create_vertex_position(int boardsize, int cell_side_length) {
    int total_cells_num = (boardsize+1)*(boardsize+1);
    int i;
    int y_cell = -1;
    for(i =0; i<total_cells_num;i++){
        if(i%(boardsize+1)==0){
            y_cell ++;
        }
        checker_vertices[i][0]= 0 - (boardsize/2)*cell_side_length + (i%(boardsize+1)*cell_side_length);
        checker_vertices[i][1]= -(0 - (boardsize/2)*cell_side_length + y_cell*cell_side_length);
        checker_vertices[i][2]= 0 ;
    }
}

void create_vertex_color(int boardsize) {
    int total_cells_num = (boardsize+1)*(boardsize+1);
    int i;
    float color1 = glc[LIGHT_GREY][0];
    float color2 = glc[DARK_GREY][0];
    
    for(i =0; i<total_cells_num;i++){
        if((i%(boardsize+1) == 0)||(i>total_cells_num-(boardsize+1))){
            checker_colors[i][0] = glc[DEBUG_RED][0];
            checker_colors[i][1] = glc[DEBUG_RED][0];
            checker_colors[i][2] = glc[DEBUG_RED][0];
        }else{
            if(i%2 != 0){
                checker_colors[i][0] = color1;
                checker_colors[i][1] = color1;
                checker_colors[i][2] = color1;
            }else{
                checker_colors[i][0] = color2;
                checker_colors[i][1] = color2;
                checker_colors[i][2] = color2;
            }
        }
    }
}

void draw_obstacle(){
    for(int i=0; i<num_obstacles; i++){
        draw_one_obstacle(Obstacles[i]);
    }
}

void draw_one_obstacle(Obstacle obs) {
    glShadeModel(GL_SMOOTH);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, obstacle_vertices);
    glColorPointer(3, GL_FLOAT, 0, goal_colors);
    
    glTranslatef((obs.loc).xyz[0], (obs.loc).xyz[1], (obs.loc).xyz[2]);
    
    glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, goal_indices);
    
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    
    glShadeModel(GL_FLAT);
    glPopMatrix();
}

void draw_checker() {
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, checker_vertices);
    glColorPointer(3, GL_FLOAT, 0, checker_colors);
    glDrawElements(GL_QUADS, BOARD_SIZE*BOARD_SIZE*4, GL_UNSIGNED_INT, checker_indices);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void gluLookAt_matrix(GLfloat eyeX, GLfloat eyeY, GLfloat eyeZ, GLfloat centerX, GLfloat centerY, GLfloat centerZ, GLfloat upX, GLfloat upY, GLfloat upZ) {
    F = assign_values(centerX - eyeX, centerY - eyeY, centerZ - eyeZ);
    up = assign_values(upX, upY, upZ);
    up = normalize_vec_struct(up);
    F = normalize_vec_struct(F);
    s = vector_product_struct(F.xyz[0],F.xyz[1],F.xyz[2],up.xyz[0],up.xyz[1],up.xyz[2]);//get s
    s = normalize_vec_struct(s);
    u = vector_product_struct(s.xyz[0],s.xyz[1],s.xyz[2],F.xyz[0],F.xyz[1],F.xyz[2]);//get u
    M_glulookat[0] = s.xyz[0]; M_glulookat[4] = s.xyz[1]; M_glulookat[8] = s.xyz[2]; M_glulookat[12] = 0;
    M_glulookat[1] = u.xyz[0]; M_glulookat[5] = u.xyz[1]; M_glulookat[9] = u.xyz[2]; M_glulookat[13] = 0;
    M_glulookat[2] = -(F.xyz[0]); M_glulookat[6] = -(F.xyz[1]); M_glulookat[10] = -(F.xyz[2]); M_glulookat[14] = 0;
    M_glulookat[3] = 0; M_glulookat[7] = 0; M_glulookat[11] = 0; M_glulookat[15] = 1;
}


void gluPerspective_self(GLfloat fovy, GLfloat aspect, GLfloat zNear, GLfloat zFar){
    GLfloat f = 1/tan(degree_to_radian(fovy/2));
    M_perspective[0] = f/aspect;
    M_perspective[1] = M_perspective[2] = M_perspective[3] = 0;
    M_perspective[4] = M_perspective[6] = M_perspective[7] = 0;
    M_perspective[5] = f;
    M_perspective[8] = M_perspective[9] = 0;
    M_perspective[10] = (zFar + zNear)/(zNear - zFar);
    M_perspective[11] = -1;
    M_perspective[12] = M_perspective[13] = M_perspective[15] = 0;
    M_perspective[14] = (2*zFar*zNear)/(zNear - zFar);
    glMultMatrixf(M_perspective);
}

void gluLookAt_self(GLfloat eyeX, GLfloat eyeY, GLfloat eyeZ, GLfloat centerX, GLfloat centerY, GLfloat centerZ, GLfloat upX, GLfloat upY, GLfloat upZ){
    gluLookAt_matrix(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, upX, upY, upZ);
    glMultMatrixf(M_glulookat);
    glTranslated(-eyeX, -eyeY, -eyeZ);
}

void draw_shadow_bird(Bird *bird, GLfloat x, GLfloat y, GLfloat z, GLfloat angle){
    if(x < VIEW_X_SIZE/2 && x > -VIEW_X_SIZE/2 && y < VIEW_X_SIZE/2 && y > -VIEW_X_SIZE/2) {
        GLfloat z_max = max(fabsf(boid_vertices[2][2]),fabsf(boid_vertices[3][2]));
        glPushMatrix();
        glColor3fv(glc[BIRD_SHADOW]);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, boid_vertices);
        glTranslatef(x, y, z_max);
        glRotatef(angle, 0, 0, 1);
    
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, boid_indices);
        glDisableClientState(GL_VERTEX_ARRAY);
        glPopMatrix();
    }
}
