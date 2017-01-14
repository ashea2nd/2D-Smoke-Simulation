//
//  fluid_sim.cpp
//  project
//
//  Created by Tony Cao on 12/4/16.
//  Copyright © 2016 Tony Cao. All rights reserved.
//

#include "fluid_sim.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <algorithm>

#define IX(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}

// Constants for buoyant force
#define ALPHA 0.1f
#define BETA 100.0f

void add_source ( int N, float * x, float * s, float dt )
{
    int i, size=(N+2)*(N+2);
    for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
    //x[100] = 2;

}

void set_bnd ( int N, int b, float * x )
{
    int i;
    
    for ( i=1 ; i<=N ; i++ ) {
        x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
        x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
        x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
        x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
        /*if ((i > N/2 - 10) && (i < N/2 + 10)) {
            x[IX(0  ,i)] = b==1 ? -1 : 1;
            x[IX(N+1,i)] = b==1 ? -1 : 1;
        }*/
    }
    x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);
    x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);
    x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);
    x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);
}

void lin_solve ( int N, int b, float * x, float * x0, float a, float c )
{
    int i, j, k;
    
    for ( k=0 ; k<20 ; k++ ) {
        FOR_EACH_CELL
        x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
        END_FOR
        set_bnd ( N, b, x );
    }
}

void diffuse ( int N, int b, float * x, float * x0, float diff, float dt )
{
    float a=dt*diff*N*N;
    lin_solve ( N, b, x, x0, a, 1+4*a );
    //lin_solve ( N, b, x, x0, a, 2+4*a );

}

void advect ( int N, int b, float * d, float * d0, float * u, float * v, float dt )
{
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;
    
    dt0 = dt*N;
    FOR_EACH_CELL
    x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
    if (x<0.5f) x=0.5f; if (x>N+0.5f) x=N+0.5f; i0=(int)x; i1=i0+1;
    if (y<0.5f) y=0.5f; if (y>N+0.5f) y=N+0.5f; j0=(int)y; j1=j0+1;
    s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
    d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
    s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
    END_FOR
    set_bnd ( N, b, d );
}

void project ( int N, float * u, float * v, float * p, float * div )
{
    int i, j;
    FOR_EACH_CELL
    div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/N;
    p[IX(i,j)] = 0;
    END_FOR
    set_bnd ( N, 0, div ); set_bnd ( N, 0, p );
    
    lin_solve ( N, 0, p, div, 1, 4 );
    
    FOR_EACH_CELL
    u[IX(i,j)] -= 0.5f*N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
    v[IX(i,j)] -= 0.5f*N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
    END_FOR
    set_bnd ( N, 1, u ); set_bnd ( N, 2, v );
}
int checkBoundaryType(int N, float * b, int i, int j) {
    if (b[IX(i + 1 , j)] &&
        b[IX(i - 1, j)] &&
        b[IX(i, j + 1)] &&
        b[IX(i, j - 1)]) {return 4;}
    else if
        (b[IX(i + 1 , j)] &&
         b[IX(i - 1, j)] &&
         b[IX(i, j - 1)]) {return 7;}
    else if
        (b[IX(i + 1 , j)] &&
         b[IX(i - 1, j)] &&
         b[IX(i, j + 1)]) {return 1;}
    else if
        (b[IX(i + 1 , j)] &&
         b[IX(i, j + 1)] &&
         b[IX(i, j - 1)]) {return 3;}
    else if
        (b[IX(i - 1 , j)] &&
         b[IX(i, j + 1)] &&
         b[IX(i, j - 1)]) {return 5;}
    else if
        (b[IX(i + 1 , j)] &&
         b[IX(i, j + 1)]) {return 0;}
    else if
        (b[IX(i + 1 , j)] &&
         b[IX(i, j - 1)]) {return 6;}
    else if
        (b[IX(i - 1 , j)] &&
         b[IX(i, j - 1)]) {return 8;}
    
    else return 2;
    
}

void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt, float * boundary, int center, float init_dens)
{
    add_source ( N, x, x0, dt );
    
    //x[98] = x[99] = x[100] = x[101] = x[102] = 2;
    //std::cout << "this is n " << N << std::endl;
    //x[IX(N/2 - 1,1)] = x[IX(N/2,1)] = x[IX(N/2 + 1,1)] = 1;
    int i, j;
    
    FOR_EACH_CELL
    if (boundary[IX(i, j)] != 0){
        /*
        x[IX(i + 1, j)] = 2*x[IX(i + 1, j)];
        x[IX(i - 1, j)] = 2* x[IX(i - 1, j)];
        x[IX(i, j + 1)] = 2*x[IX(i, j + 1)];
        x[IX(i, j - 1)] = 2*x[IX(i, j - 1)];
        x[IX(i + 1, j + 1)] = 2*x[IX(i + 1, j + 1)];
        x[IX(i + 1, j - 1)] = 2*x[IX(i + 1, j - 1)];
        x[IX(i - 1, j + 1)] = 2*x[IX(i - 1, j + 1)];
        x[IX(i - 1, j - 1)] = 2*x[IX(i - 1, j - 1)];
         */
        x[IX(i, j)] = 0;
    }
    END_FOR
    
    for ( int i= fmax(center - 10, 0); i<=fmin(center + 10, N); i++ ) {
        x[IX(i,1)] = init_dens;

    }
    SWAP ( x0, x ); diffuse ( N, 0, x, x0, diff, dt );
    SWAP ( x0, x ); advect ( N, 0, x, x0, u, v, dt );

}

void temp_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt )
{
  add_source ( N, x, x0, dt );
  SWAP ( x0, x ); diffuse ( N, 0, x, x0, diff, dt );
  SWAP ( x0, x ); advect ( N, 0, x, x0, u, v, dt );
}



void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt, float * temp, float * dens, float * boundary, int center, int init_V)
{
    //add_source ( int N, float * gridToBeUpdated, float * currentGrid, float time_step )
    //v[98] = v[99] = v[100] = v[101] = v[102] = 2;
    //u[] = u[99] = u[100] = u[101] = u[102] = 2;
    //v[IX(N/2 - 1,1)] = v[IX(N/2,1)] = v[IX(N/2 + 1,1)] = 2.8;
    for ( int i= fmax(center - 10, 0); i<=fmin(center + 10, N); i++ ) {
        v[IX(i,1)] = init_V;
    }
    add_source ( N, u, u0, dt ); add_source ( N, v, v0, dt );
    SWAP ( u0, u ); diffuse ( N, 1, u, u0, visc, dt );
    SWAP ( v0, v ); diffuse ( N, 2, v, v0, visc, dt );
    project ( N, u, v, u0, v0 );
    SWAP ( u0, u ); SWAP ( v0, v );
    advect ( N, 1, u, u0, u0, v0, dt ); advect ( N, 2, v, v0, u0, v0, dt );
    project ( N, u, v, u0, v0 );
  
    for (int i = 0; i < (N + 2) * (N + 2); i++) {
      v[i] += fmax(-ALPHA * dens[i] + BETA * temp[i], 0);
    }
    int i, j;
    FOR_EACH_CELL
    if (boundary[IX(i, j)] != 0) {
        int boundType = checkBoundaryType(N, boundary, i, j);
        switch (boundType)
        {
            case 0:
                v[IX(i, j)] = -1.0*(v[IX(i - 1, j)] +
                                    v[IX(i, j - 1)] -
                                    v[IX(i - 1, j - 1)])/3.0f;
                u[IX(i, j)] = -1.0*(u[IX(i - 1, j)] +
                                    u[IX(i, j - 1)] -
                                    u[IX(i - 1, j - 1)])/3.0f;
                break;
            case 2:
                v[IX(i, j)] = -1.0*(v[IX(i + 1, j)] +
                                    v[IX(i, j - 1)] -
                                    v[IX(i + 1, j - 1)])/3.0f;
                u[IX(i, j)] = -1.0*(u[IX(i + 1, j)] +
                                    u[IX(i, j - 1)] -
                                    u[IX(i + 1, j - 1)])/3.0f;
                break;
            case 6:
                v[IX(i, j)] = -1.0*(v[IX(i - 1, j)] +
                                    v[IX(i, j + 1)] -
                                    v[IX(i - 1, j + 1)])/3.0f;
                u[IX(i, j)] = -1.0*(u[IX(i - 1, j)] +
                                    u[IX(i, j + 1)] -
                                    u[IX(i - 1, j + 1)])/3.0f;
                break;
            case 8:
                v[IX(i, j)] = -1.0*(v[IX(i + 1, j)] +
                                    v[IX(i, j + 1)] -
                                    v[IX(i + 1, j + 1)])/3.0f;
                u[IX(i, j)] = -1.0*(u[IX(i + 1, j)] +
                                    u[IX(i, j + 1)] -
                                    u[IX(i + 1, j + 1)])/3.0f;

                break;
            case 1:
                v[IX(i, j)] = -1*v[IX(i, j - 1)];
                //u[IX(i, j)] = -1*u[IX(i, j - 1)];
                break;
            case 7:
                v[IX(i, j)] = -1*v[IX(i, j + 1)];
                //u[IX(i, j)] = -1*u[IX(i, j + 1)];
                break;
            case 3:
                //v[IX(i, j)] = -1*v[IX(i - 1, j)];
                u[IX(i, j)] = -1*u[IX(i - 1, j)];
                break;
            case 5:
                //v[IX(i, j)] = -1*v[IX(i + 1, j)];
                u[IX(i, j)] = -1*u[IX(i + 1, j)];
                break;
            case 4:
                v[IX(i, j)] = 0;
                u[IX(i, j)] = 0;
                break;
        }
    };
    END_FOR;
}

