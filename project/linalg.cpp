//
//  linalg.cpp
//  project
//
//  Created by Tony Cao on 12/6/16.
//  Copyright © 2016 Tony Cao. All rights reserved.
//

#include "linalg.hpp"
#include <valarray>

vec conjugate_gradient(float * A, int N, float ro, float tol, vec d) {
  matrix A_ = toMatrix(A, N);
  vec p;
  vec r = d;
  vec z = applyPreconditioner(r);
  vec s = z;
  float sigma = dot(z, r);
  int iteration_count = 0;
  
  while (iteration_count < 100) {
    z = matrixMultiply(A_, s);
    float alpha = ro / dot(z, s);
    p += alpha * s;
    r -= alpha * s;
    if (r.max() <= tol) {
      return p;
    }
    z = applyPreconditioner(r);
    float sigma_new = dot(z, r);
    float beta = sigma_new / ro;
    s = z + beta * s;
    iteration_count++;
  }
  
  return p;
}

// Converts a 1d array representation of a 2d grid into a vector representation
matrix toMatrix(float * A, int N) {
  matrix m(N);
  for (int i = 0; i < N; i++) {
    vec row(N);
    for (int j = 0; j < N; j++) {
      row[j] = A[i + N * j];
    }
    m[i] = row;
  }
  return m;
}

vec matrixMultiply(matrix & A, vec & d) {
  vec out(d.size());
  for (int i = 0; i < A.size(); i++) {
    vec row = A[i];
    float val = 0;
    for (int j = 0; j < row.size(); j++) {
      val += row[j] * d[j];
    }
    out[i] = val;
  }
  return out;
}

float dot(vec & a, vec & b) {
  return (a*b).sum();
}

vec applyPreconditioner(vec r) {
  float tau = .97;
  return r;
}

matrix findE(matrix A) {
  return A;
}
