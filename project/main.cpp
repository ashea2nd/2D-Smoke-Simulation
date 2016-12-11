//
//  main.cpp
//  project
//
//  Created by Tony Cao on 12/4/16.
//  Copyright © 2016 Tony Cao. All rights reserved.
//

#include <stdlib.h>
#include <stdio.h>
#include "vecmath.h"
#include <iostream>
#include <glfw3.h>
#include "gl.h"
#include <OpenGL/OpenGL.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include "linalg.hpp"
#include <math.h>
#include <algorithm>


#define IX(i,j) ((i)+(N+2)*(j))
#define SMOKE_MODE 0
#define VELOCITY_MODE 1
#define TEMPERATURE_MODE 2
#define NUM_MODES 3

/* external definitions (from solver.c) */

extern void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt, float * boundary, int center, float init_dens);
extern void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt, float * temp, float * dens, float * boundary, int center, int init_V);
extern void temp_step ( int N, float * temp, float * temp0, float * u, float * v, float diff, float dt );
//extern void draw_boundary ( int N, float * boundary, float * u, float * v, float diff, float dt );


/* global variables */

static int N;
static float dt, diff, visc;
static float force, smoke_source, temp_source;
static int display_mode;
static int add_mode;
static int center;
static int init_V;
static float init_dens;

static float * u, * v, * u_prev, * v_prev;
static float * dens, * dens_prev;
static float * red_dens, * red_dens_prev;
static float * green_dens, * green_dens_prev;
static float * blue_dens, * blue_dens_prev;
static float * custom_dens, * custom_dens_prev;


static float * temp, * temp_prev;
static float * boundary;
static int block_width;
static int block_height;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;
static int color_mode = 0;

static float total_green, total_red, total_blue = 0.0f;
static float MAX_COLOR = 255.0f;

/*
 ----------------------------------------------------------------------
 free/clear/allocate simulation data
 ----------------------------------------------------------------------
 */


static void free_data ( void )
{
  if ( u ) free ( u );
  if ( v ) free ( v );
  if ( u_prev ) free ( u_prev );
  if ( v_prev ) free ( v_prev );
  if ( dens ) free ( dens );
  if ( dens_prev ) free ( dens_prev );
  if ( red_dens ) free ( red_dens );
  if ( red_dens_prev ) free ( red_dens_prev );
  if ( green_dens ) free ( green_dens );
  if ( green_dens_prev ) free ( green_dens_prev );
  if (blue_dens) free (blue_dens);
  if (blue_dens_prev) free (blue_dens_prev);
  if (custom_dens) free (custom_dens);
  if (custom_dens_prev) free (custom_dens_prev);

  if ( temp ) free ( temp );
  if ( temp_prev ) free ( temp_prev );
  if ( boundary ) free ( boundary );
}

static void clear_data ( void )
{
    int i, size=(N+2)*(N+2);
    
    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = red_dens[i] = red_dens_prev[i] =
        green_dens[i] = green_dens_prev[i] = temp[i] = temp_prev[i] = boundary[i] = blue_dens[i] = blue_dens_prev[i] =
            custom_dens[i] = custom_dens_prev[i] = 0.0f;
        //if ( i > N/2 - 15 && i < N/2 + 15){
          //  dens_prev[i] = 10.0f;
            //v_prev[i] = 10.0;
        //}
    }
}

static int allocate_data ( void )
{
  int size = (N+2)*(N+2);
  
  u			= (float *) malloc ( size*sizeof(float) );
  v			= (float *) malloc ( size*sizeof(float) );
  u_prev		= (float *) malloc ( size*sizeof(float) );
  v_prev		= (float *) malloc ( size*sizeof(float) );
  dens		= (float *) malloc ( size*sizeof(float) );
  dens_prev	= (float *) malloc ( size*sizeof(float) );
  temp		= (float *) malloc ( size*sizeof(float) );
  temp_prev	= (float *) malloc ( size*sizeof(float) );
  boundary  = (float *) malloc ( size*sizeof(float) );
  red_dens  = (float *) malloc ( size*sizeof(float) );
  red_dens_prev = (float *) malloc ( size*sizeof(float) );
  green_dens = (float *) malloc ( size*sizeof(float) );
  green_dens_prev = (float *) malloc ( size*sizeof(float) );
  blue_dens = (float *) malloc ( size*sizeof(float) );
  blue_dens_prev = (float *) malloc ( size*sizeof(float) );
  custom_dens = (float *) malloc ( size*sizeof(float) );
  custom_dens_prev = (float *) malloc ( size*sizeof(float) );

  
  if ( !u || !v || !u_prev || !v_prev || !dens || !dens_prev || !boundary) {
    fprintf ( stderr, "cannot allocate data\n" );
    return ( 0 );
  }
  
  return ( 1 );
}


/*
 ----------------------------------------------------------------------
 OpenGL specific drawing routines
 ----------------------------------------------------------------------
 */

static void pre_display ( void )
{
  glViewport ( 0, 0, win_x, win_y );
  glMatrixMode ( GL_PROJECTION );
  glLoadIdentity ();
  gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
  glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
  glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
  glutSwapBuffers ();
}

static std::string get_mode_string ( int mode ) {
  if (mode == VELOCITY_MODE) return "velocity";
  else if (mode == TEMPERATURE_MODE) return "temp";
  else if (mode == SMOKE_MODE) return "smoke";
  else  return "unknown";
}

static void draw_modes() {
  glColor3f ( 0.0f, 1.0f, 0.0f );
  glRasterPos2f(.005, .98);
  for (char c : "Display " + get_mode_string(display_mode)) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
  }
  std::string init_dens_char = std::to_string(init_dens);
  glRasterPos2f(.005, .95);
  for (char c : "Source Density (X Z): " + init_dens_char) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
  }
  std::string init_V_char = std::to_string(init_V);
  glRasterPos2f(.005, .92);
  for (char c : "Source Velocity (Up Down): " + init_V_char) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
  }
    
  std::string block_height_char = std::to_string(block_height);
  glRasterPos2f(.005, .89);
  for (char c : "Block Height (W S): " + block_height_char) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
  }
  std::string block_width_char = std::to_string(block_width);
  glRasterPos2f(.005, .86);
  for (char c : "Block Width (A D): " + block_width_char) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
  }

  glRasterPos2f(.83, .98);
  for (char c : "Adding " + get_mode_string(add_mode)) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
  }
}


static void draw_velocity ( void )
{
    int i, j;
    float x, y, h;
    
    h = 1.0f/N;
    
    glColor3f ( 1.0f, 1.0f, 1.0f );
    glLineWidth ( 1.0f );
    
    glBegin ( GL_LINES );
    
    for ( i=1 ; i<=N ; i++ ) {
        x = (i-0.5f)*h;
        for ( j=1 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;
            
            glVertex2f ( x, y );
            glVertex2f ( x+u[IX(i,j)], y+v[IX(i,j)] );
        }
    }
    
    glEnd ();
}

static void draw_density ( void )
{
    int i, j;
    //float x, y, h, d00, d01, d10, d11, r00, r01, r10, r11, g00, g01, g10, g11, isBoundary;
    float x, y, h, d00_red, d01_red, d10_red, d11_red, isBoundary;
    float d00_green, d01_green, d10_green, d11_green,
            d00_blue, d01_blue, d10_blue, d11_blue,
             d00_custom, d01_custom, d10_custom, d11_custom;
    
    h = 1.0f/N;
    
    glBegin ( GL_QUADS );
    
    for ( i=0 ; i<=N ; i++ ) {
        x = (i-0.5f)*h;
        for ( j=0 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;
            
            isBoundary = boundary[IX(i,j)];
            /*
            d00 = dens[IX(i,j)];
            d01 = dens[IX(i,j+1)];
            d10 = dens[IX(i+1,j)];
            d11 = dens[IX(i+1,j+1)];
            
            r00 = red_dens[IX(i,j)];
            r01 = red_dens[IX(i,j+1)];
            r10 = red_dens[IX(i+1,j)];
            r11 = red_dens[IX(i+1,j+1)];
            
            g00 = green_dens[IX(i,j)];
            g01 = green_dens[IX(i,j+1)];
            g10 = green_dens[IX(i+1,j)];
            g11 = green_dens[IX(i+1,j+1)];
            */
            
            d00_red = red_dens[IX(i,j)];
            d01_red = red_dens[IX(i,j+1)];
            d10_red = red_dens[IX(i+1,j)];
            d11_red = red_dens[IX(i+1,j+1)];
            d00_blue = blue_dens[IX(i,j)];
            d01_blue = blue_dens[IX(i,j+1)];
            d10_blue = blue_dens[IX(i+1,j)];
            d11_blue = blue_dens[IX(i+1,j+1)];
            d00_green = green_dens[IX(i,j)];
            d01_green = green_dens[IX(i,j+1)];
            d10_green = green_dens[IX(i+1,j)];
            d11_green = green_dens[IX(i+1,j+1)];
            d00_custom = custom_dens[IX(i,j)];
            d01_custom = custom_dens[IX(i,j+1)];
            d10_custom = custom_dens[IX(i+1,j)];
            d11_custom = custom_dens[IX(i+1,j+1)];
            //glColor3f assigns the color
            //dxx correspond to color values
            //x+h and y+h means that each added source / density value corresponds to 4 grid regions
            //same 3 values = white
            //glColor3f ( d00, d00, d00 ); glVertex2f ( x, y );
            //glColor3f ( d10, d10, d10 ); glVertex2f ( x+h, y );
            //glColor3f ( d11, d11, d11 ); glVertex2f ( x+h, y+h );
            //glColor3f ( d01, d01, d01 ); glVertex2f ( x, y+h );
            if (isBoundary) {
                glColor3f ( 0.75, 0.75, 0.75 ); glVertex2f ( x, y );
                glColor3f ( 0.75, 0.75, 0.75 ); glVertex2f ( x+h, y );
                glColor3f ( 0.75, 0.75, 0.75 ); glVertex2f ( x+h, y+h );
                glColor3f ( 0.75, 0.75, 0.75 ); glVertex2f ( x, y+h );
                
            } else {
                /*
                int thresh = 0.7f;
                if (d00 > thresh) {
                    glColor3f ( total_red, total_green, d00); glVertex2f ( x, y );
                } else {
                    glColor3f ( 0, 0, d00 ); glVertex2f ( x, y );
                }
                
                if (d10 > thresh) {
                    glColor3f ( total_red, total_green, d10 ); glVertex2f ( x+h, y );
                } else {
                    glColor3f ( 0, 0, d10 ); glVertex2f ( x, y );
                }
                
                if (d11 > thresh) {
                    glColor3f ( total_red, total_green, d11 ); glVertex2f ( x+h, y+h );
                } else {
                    glColor3f ( 0, 0, d11 ); glVertex2f ( x, y );
                }
                
                if (d01 > thresh) {
                    glColor3f ( total_red, total_green, d01 ); glVertex2f ( x, y+h );
                } else {
                    glColor3f ( 0, 0, d01 ); glVertex2f ( x, y );
                }
                */
                /*
                glColor3f ( 0, 0, r00); glVertex2f ( x, y );
                glColor3f ( 0, 0, r10 ); glVertex2f ( x+h, y );
                glColor3f ( 0, 0, r11); glVertex2f ( x+h, y+h );
                glColor3f ( 0, 0, r01); glVertex2f ( x, y+h );
                */
                
                /*
                glColor3f ( r00, g00, d00); glVertex2f ( x, y );
                glColor3f ( r10, g10, d10); glVertex2f ( x+h, y );
                glColor3f ( r11, g11, d11); glVertex2f ( x+h, y+h );
                glColor3f ( r01, g01, d01); glVertex2f ( x, y+h );
                */
                /*
                glColor3f ( d00_red*total_red, d00_green*total_green, d00_blue*total_blue ); glVertex2f ( x, y );
                glColor3f ( d10_red*total_red, d10_green*total_green, d10_blue*total_blue ); glVertex2f ( x+h, y );
                glColor3f ( d11_red*total_red, d11_green*total_green, d11_blue*total_blue ); glVertex2f ( x+h, y+h );
                glColor3f ( d01_red*total_red, d01_green*total_green, d01_blue*total_blue ); glVertex2f ( x, y+h );
                */
                
                //USE THIS ONE
                /*
                glColor3f ( d00_red, d00_green, d00_blue); glVertex2f ( x, y );
                glColor3f ( d10_red, d10_green, d10_blue); glVertex2f ( x+h, y );
                glColor3f ( d11_red, d11_green, d11_blue); glVertex2f ( x+h, y+h );
                glColor3f ( d01_red, d01_green, d01_blue); glVertex2f ( x, y+h );
                */
                
                
                glColor3f ( d00_red + d00_custom*total_red, d00_green + d00_custom*total_green, d00_blue + d00_custom*total_blue); glVertex2f ( x, y );
                glColor3f ( d10_red + d10_custom*total_red, d10_green + d10_custom*total_green, d10_blue + d10_custom*total_blue); glVertex2f ( x+h, y );
                glColor3f ( d11_red + d11_custom*total_red, d11_green + d11_custom*total_green, d11_blue + d11_custom*total_blue); glVertex2f ( x+h, y+h );
                glColor3f ( d01_red + d01_custom*total_red, d01_green + d01_custom*total_green, d01_blue + d01_custom*total_blue); glVertex2f ( x, y+h );
                
                
                
                /*
                glColor3f ( d00*total_red, d00*total_green, d00*total_blue ); glVertex2f ( x, y );
                glColor3f ( d10*total_red, d10*total_green, d10*total_blue ); glVertex2f ( x+h, y );
                glColor3f ( d11*total_red, d11*total_green, d11*total_blue ); glVertex2f ( x+h, y+h );
                glColor3f ( d01*total_red, d01*total_green, d01*total_blue ); glVertex2f ( x, y+h );
                */
                /*
                glColor3f ( 0, 0, d00 ); glVertex2f ( x, y );
                glColor3f ( 0, 0, d10 ); glVertex2f ( x+h, y );
                glColor3f ( 0, 0, d11 ); glVertex2f ( x+h, y+h );
                glColor3f ( 0, 0, d01 ); glVertex2f ( x, y+h );
                */
            }
            //}
        }
    }
    
    glEnd ();
}

static void draw_temperature ( void )
{
    int i, j;
    float x, y, h, d00, d01, d10, d11;
    
    h = 1.0f/N;
    
    glBegin ( GL_QUADS );
    
    for ( i=0 ; i<=N ; i++ ) {
        x = (i-0.5f)*h;
        for ( j=0 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;
            
            d00 = temp[IX(i,j)] * 100;
            d01 = temp[IX(i,j+1)] * 100;
            d10 = temp[IX(i+1,j)] * 100;
            d11 = temp[IX(i+1,j+1)] * 100;
            
            glColor3f ( d00, 0, 0 ); glVertex2f ( x, y );
            glColor3f ( d10, 0, 0 ); glVertex2f ( x+h, y );
            glColor3f ( d11, 0, 0 ); glVertex2f ( x+h, y+h );
            glColor3f ( d01, 0, 0 ); glVertex2f ( x, y+h );

        }
    }
    
    glEnd ();
}



/*
 ----------------------------------------------------------------------
 relates mouse movements to forces sources
 ----------------------------------------------------------------------
 */


static void get_from_UI ( float * d, float * u, float * v, float * temp, int curr_color_mode )
{
    int i, j, size = (N+2)*(N+2);
    
    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = d[i] = temp[i] = 0.0f;
    }
    if (!mouse_down[0] && !mouse_down[2]) return;
    
    i = (int)((       mx /(float)win_x)*N+1);
    j = (int)(((win_y-my)/(float)win_y)*N+1);
    if ( i<1 || i>N || j<1 || j>N ) return;
    
    if (mouse_down[0]){
    
        //i = (int)((       mx /(float)win_x)*N+1);
        //j = (int)(((win_y-my)/(float)win_y)*N+1);
    
        //if ( i<1 || i>N || j<1 || j>N ) return;
    
        if ( add_mode == VELOCITY_MODE ) {
            if (curr_color_mode == 3){
                u[IX(i,j)] = force * (mx-(omx));
                v[IX(i,j)] = force * ((omy)-my);
                std::cout << mx << omx << std::endl;
                std::cout << my << omy << std::endl;
                omx = mx;
                omy = my;

            }
        }
    
        else if ( add_mode == SMOKE_MODE ) {
            if (color_mode == curr_color_mode){
                d[IX(i,j)] = smoke_source;
            }
        }

        else if ( add_mode == TEMPERATURE_MODE ) {
            temp[IX(i, j)] = temp_source;
        }
    
        //omx = mx;
        //omy = my;
    }
    if (mouse_down[2]) {
        for (int start = 0; start < block_width; start++) {
            for (int end = 0; end < block_height; end++) {
                boundary[IX(i + start, j + end)] = 1.0f;
            }
        }
    }
    return;
}

/*
 ----------------------------------------------------------------------
 GLUT callback routines
 ----------------------------------------------------------------------
 */
static void special_key_func ( int key, int x, int y )
{
    switch(key) {
        case GLUT_KEY_LEFT:
            center = fmax(center - 3, 0);
            break;
        case GLUT_KEY_RIGHT:
            center = fmin(center + 3, N);
            break;
        case GLUT_KEY_UP:
            init_V = fmin(init_V + 1, 40);
            break;
        case GLUT_KEY_DOWN:
            init_V = fmax(init_V - 1, 1);
            break;
    }
}

bool isDown = false;
static void key_func ( unsigned char key, int x, int y )
{
    //std::cout << key << std::endl;
  switch ( key )
  {
    case 'c':
    case 'C':
      clear_data ();
      break;
        
    case 'q':
    case 'Q':
      free_data ();
      exit ( 0 );
      break;
        
    case 'v':
    case 'V':
          isDown = true;
          //std::cout << isDown << std::endl;
      display_mode = (display_mode + 1) % NUM_MODES;
      break;
      
    case 't':
    case 'T':
      add_mode = (add_mode + 1) % NUM_MODES;
      break;
    case 'i':
    case 'I':
          total_red = fmin(total_red + 0.03, 1);
          std::cout << "Red: " << total_red << std::endl;
          break;
    case 'o':
    case 'O':
          total_green = fmin(total_green + 0.03, 1);
          std::cout << "Green: " << total_green << std::endl;
          break;
    case 'p':
    case 'P':
          total_blue = fmin(total_blue + 0.03, 1);
          std::cout << "Blue: " << total_blue << std::endl;

          break;
    case 'j':
    case 'J':
          total_red = fmax(total_red - 0.03, 0);
          std::cout << "Red: " << total_red << std::endl;
          break;
    case 'k':
    case 'K':
          total_green = fmax(total_green - 0.03, 0);
          std::cout << "Green: " << total_green << std::endl;
          break;
    case 'l':
    case 'L':
          total_blue = fmax(total_blue - 0.03, 0);
          std::cout << "Blue: " << total_blue << std::endl;
          break;
    case 'z':
    case 'Z':
          init_dens = fmax(init_dens - 0.2, 0.2);
          break;
    case 'x':
    case 'X':
          init_dens = fmin(init_dens + 0.2, 3);
          break;
    case 'A':
    case 'a':
          block_width = fmax(block_width - 1, 3);
          break;
    case 'd':
    case 'D':
          block_width = fmin(block_width + 1, 20);
          break;
    case 'w':
    case 'W':
          block_height = fmin(block_height + 1, 20);
          break;
    case 's':
    case 'S':
          block_height = fmax(block_height - 1, 3);
          break;
    case 'm':
    case 'M':
          color_mode = fmod(color_mode + 1, 4);
          break;
  }
}

static void mouse_func ( int button, int state, int x, int y )
{
    //std::cout << "Mouse is not moving" << std::endl;
    //omx = mx = x;
    //omx = my = y;
    mx = x;
    my = y;
    mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
    //std::cout << "Mouse is moving" << std::endl;
    mx = x;
    my = y;
}

static void reshape_func ( int width, int height )
{
    glutSetWindow ( win_id );
    glutReshapeWindow ( width, height );
    
    win_x = width;
    win_y = height;
}

static void idle_func ( void )
{
    float * d;
    float r_init, b_init, g_init, c_init = 0;
    if (color_mode == 0) {
        //d = dens_prev;
        r_init = init_dens;
    } else if (color_mode == 1) {
        //d = blue_dens_prev;
        b_init = init_dens;
    } else if (color_mode == 2) {
        //d = green_dens_prev;
        g_init = init_dens;
    } else {
        c_init = init_dens;
    }
    
    get_from_UI ( red_dens_prev, u_prev, v_prev, temp_prev, 0);
    get_from_UI ( blue_dens_prev, u_prev, v_prev, temp_prev, 1);
    get_from_UI ( green_dens_prev, u_prev, v_prev, temp_prev, 2);
    get_from_UI ( custom_dens_prev, u_prev, v_prev, temp_prev, 3);

    temp_step ( N, temp, temp_prev, u, v, diff, dt );
    vel_step ( N, u, v, u_prev, v_prev, visc, dt, temp, dens, boundary, center, init_V);
    //dens_step ( N, dens, dens_prev, u, v, diff, dt, boundary, center, init_dens);
    
    dens_step ( N, red_dens, red_dens_prev, u, v, diff, dt, boundary, center, r_init);
    dens_step ( N, blue_dens, blue_dens_prev, u, v, diff, dt, boundary, center, b_init);
    dens_step ( N, green_dens, green_dens_prev, u, v, diff, dt, boundary, center, g_init);
    dens_step ( N, custom_dens, custom_dens_prev, u, v, diff, dt, boundary, center, c_init);

    
    glutSetWindow ( win_id );
    glutPostRedisplay ();
}

static void display_func ( void )
{
    pre_display ();

    if ( display_mode == VELOCITY_MODE ) draw_velocity ();
    else if ( display_mode == SMOKE_MODE ) draw_density ();
    else if (display_mode == TEMPERATURE_MODE ) draw_temperature();
    
    draw_modes();

    post_display ();
}


/*
 ----------------------------------------------------------------------
 open_glut_window --- open a glut compatible window and set callbacks
 ----------------------------------------------------------------------
 */
static void open_glut_window ( void )
{
    glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );
    
    glutInitWindowPosition ( 0, 0 );
    glutInitWindowSize ( win_x, win_y );
    win_id = glutCreateWindow ( "Alias | wavefront" );
    
    glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
    glClear ( GL_COLOR_BUFFER_BIT );
    glutSwapBuffers ();
    glClear ( GL_COLOR_BUFFER_BIT );
    glutSwapBuffers ();
    
    pre_display ();
    
    glutKeyboardFunc ( key_func );
    glutSpecialFunc ( special_key_func );
    glutMouseFunc ( mouse_func );
    glutMotionFunc ( motion_func );
    glutReshapeFunc ( reshape_func );
    glutIdleFunc ( idle_func );
    glutDisplayFunc ( display_func );
}


/*
 ----------------------------------------------------------------------
 main --- main routine
 ----------------------------------------------------------------------
 */

int main ( int argc, char ** argv )
{
    glutInit ( &argc, argv );

    if ( argc != 1 && argc != 6 ) {
        fprintf ( stderr, "usage : %s N dt diff visc force source\n", argv[0] );
        fprintf ( stderr, "where:\n" );\
        fprintf ( stderr, "\t N      : grid resolution\n" );
        fprintf ( stderr, "\t dt     : time step\n" );
        fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
        fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
        fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
        fprintf ( stderr, "\t smoke source : amount of smoke that will be deposited\n" );
        fprintf ( stderr, "\t temp source : amount of temperature that will be deposited\n" );
        exit ( 1 );
    }
    
    if ( argc == 1 ) {
        N = 200;
        //dt = 0.1f;
        center = N/2;
        init_V = 1;
        init_dens = 0.8;
        block_width = 3;
        block_height = 3;
        dt = 40.0f/N;
        diff = 0.0f;
        visc = 0.0f;
        force = 2.0f;
        smoke_source = 100.0f;
        temp_source = 0.5f;
        fprintf ( stderr, "Using defaults : N=%d dt=%g diff=%g visc=%g force = %g smoke_source=%g temp_source=%g\n",
                 N, dt, diff, visc, force, smoke_source, temp_source );
    } else {
        N = atoi(argv[1]);
        dt = atof(argv[2]);
        diff = atof(argv[3]);
        visc = atof(argv[4]);
        force = atof(argv[5]);
        smoke_source = atof(argv[6]);
        temp_source = atof(argv[7]);
    }
    
    printf ( "\n\nHow to use this demo:\n\n" );
    printf ( "\t Toggle addition mode with the 't' key\n");
    printf ( "\t Add quantities with the left mouse button\n" );
    printf ( "\t Toggle display with the 'v' key\n" );
    printf ( "\t Cleagr the simulation by pressing the 'c' key\n" );
    printf ( "\t Hold 'i', 'o', or 'p' to increase r,g,b color channels, respectively\n" );
    printf ( "\t Hold 'j', 'k', or 'l' to decrease r,g,b color channels, respectively\n" );
    printf ( "\t Use 'z' and 'x' to toggle source density" );
    printf ( "\t Use 'a' and 's' to toggle block size " );
    printf ( "\t Use arrow keys to move source around\n");
    printf ( "\t Right click to add a boundary\n");
    printf ( "\t Quit by pressing the 'q' key\n" );
    
    display_mode = 0;
    add_mode = 0;
    
    if ( !allocate_data () ) exit ( 1 );
    clear_data ();
    
    win_x = 512;
    win_y = 512;
    open_glut_window ();
    
    glutMainLoop ();
    
    exit ( 0 );
}
