//
//  linalg.hpp
//  project
//
//  Created by Tony Cao on 12/6/16.
//  Copyright © 2016 Tony Cao. All rights reserved.
//

#ifndef linalg_hpp
#define linalg_hpp

//#include <stdio.h>
#include <valarray>
#include <functional>

#define matrix std::valarray<std::valarray<float>>
#define vec std::valarray<float>

void conjugate_gradient(float * A, vec d);

matrix toMatrix(float * A, int N);

vec matrixMultiply(matrix & A, vec & d);

float dot(vec & a, vec & b);

vec applyPreconditioner(vec r);

#endif /* linalg_hpp */
