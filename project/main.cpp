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

#define IX(i,j) ((i)+(N+2)*(j))
#define SMOKE_MODE 0
#define VELOCITY_MODE 1
#define TEMPERATURE_MODE 2
#define NUM_MODES 3

/* external definitions (from solver.c) */

extern void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt );
extern void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt, float * temp, float * dens );
extern void temp_step ( int N, float * temp, float * temp0, float * u, float * v, float diff, float dt );

/* global variables */

static int N;
static float dt, diff, visc;
static float force, smoke_source, temp_source;
static int display_mode;
static int add_mode;

static float * u, * v, * u_prev, * v_prev;
static float * dens, * dens_prev;
static float * temp, * temp_prev;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;


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
  if ( temp ) free ( temp );
  if ( temp_prev ) free ( temp_prev );
}

static void clear_data ( void )
{
    int i, size=(N+2)*(N+2);
    
    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
        if ( i > N/2 - 15 && i < N/2 + 15){
            dens_prev[i] = 10.0f;
            v_prev[i] = 10.0;
        }
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
  
  if ( !u || !v || !u_prev || !v_prev || !dens || !dens_prev ) {
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
    float x, y, h, d00, d01, d10, d11;
    
    h = 1.0f/N;
    
    glBegin ( GL_QUADS );
    
    for ( i=0 ; i<=N ; i++ ) {
        x = (i-0.5f)*h;
        for ( j=0 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;
            
            d00 = dens[IX(i,j)];
            //d00 = 100;
            d01 = dens[IX(i,j+1)];
            d10 = dens[IX(i+1,j)];
            d11 = dens[IX(i+1,j+1)];
            //glColor3f assigns the color
            //dxx correspond to color values
            //x+h and y+h means that each added source / density value corresponds to 4 grid regions
            //same 3 values = white
            glColor3f ( d00, d00, d00 ); glVertex2f ( x, y );
            glColor3f ( d10, d10, d10 ); glVertex2f ( x+h, y );
            glColor3f ( d11, d11, d11 ); glVertex2f ( x+h, y+h );
            glColor3f ( d01, d01, d01 ); glVertex2f ( x, y+h );
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


static void get_from_UI ( float * d, float * u, float * v, float * temp )
{
    int i, j, size = (N+2)*(N+2);
    
    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = d[i] = temp[i] = 0.0f;
    }
    
    if ( !mouse_down[0]) return;
    
    i = (int)((       mx /(float)win_x)*N+1);
    j = (int)(((win_y-my)/(float)win_y)*N+1);
    
    if ( i<1 || i>N || j<1 || j>N ) return;
    
    if ( add_mode == VELOCITY_MODE ) {
        u[IX(i,j)] = force * (mx-omx);
        v[IX(i,j)] = force * (omy-my);
    }
    
    else if ( add_mode == SMOKE_MODE ) {
        d[IX(i,j)] = smoke_source;
    }

    else if ( add_mode == TEMPERATURE_MODE ) {
        temp[IX(i, j)] = temp_source;
    }
    
    omx = mx;
    omy = my;
    
    return;
}

/*
 ----------------------------------------------------------------------
 GLUT callback routines
 ----------------------------------------------------------------------
 */

static void key_func ( unsigned char key, int x, int y )
{
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
      display_mode = (display_mode + 1) % NUM_MODES;
      break;
      
    case 't':
    case 'T':
      add_mode = (add_mode + 1) % NUM_MODES;
      break;
  }
}

static void mouse_func ( int button, int state, int x, int y )
{
    std::cout << "Mouse is not moving" << std::endl;
    omx = mx = x;
    omx = my = y;
    
    mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
    std::cout << "Mouse is moving" << std::endl;
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
    get_from_UI ( dens_prev, u_prev, v_prev, temp_prev );
    vel_step ( N, u, v, u_prev, v_prev, visc, dt, temp, dens );
    dens_step ( N, dens, dens_prev, u, v, diff, dt );
    temp_step ( N, temp, temp_prev, u, v, diff, dt );

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
        dt = 100.0f/N;
        diff = 0.0f;
        visc = 0.0f;
        force = 5.0f;
        smoke_source = 100.0f;
        temp_source = 1.0f;
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