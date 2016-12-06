//
//  MAC.h
//  project
//
//  Created by Tony Cao on 12/4/16.
//  Copyright © 2016 Tony Cao. All rights reserved.
//
#include <vector>
#include <assert.h>

#ifndef MAC_h
#define MAC_h

using namespace std;
struct GridPoint {
  float velocity;
  float temperature;
  float concentration;
  float up;
  float down;
  float left;
  float right;
};

class MAC {
public:
  MAC(int length) {
    for (int i = 0; i < length; ++i) {
      vector<GridPoint> row;
      for (int j = 0; j < length; ++j) {
        row.push_back(GridPoint{0, 0, 0, 0, 0, 0, 0});
      }
      grid.push_back(row);
    }
  }
  
  MAC(int height, int width) {
    for (int i = 0; i < height; ++i) {
      vector<GridPoint> row;
      for (int j = 0; j < width; ++j) {
        row.push_back(GridPoint{0, 0, 0, 0, 0, 0, 0});
      }
      grid.push_back(row);
    }
  }
  
  const GridPoint getGridPoint(int row, int col) {
    assert(row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    return r[col];
  }
  
  const void setGridPoint(GridPoint newPoint, int row, int col) {
    assert(row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    r[col] = newPoint;
  }
  
  const void setConcentration(int row, int col, float concentration){
    assert (row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    r[col].concentration = concentration;
  }
  
  const float getVelocity(int row, int col) {
    assert(row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    return r[col].velocity;
  }
  
  const float getTemperature(int row, int col) {
    assert(row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    return r[col].temperature;      
  }
  
  const float getConcentration(int row, int col) {
    assert(row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    return r[col].concentration;
  }
  
  const float getUp(int row, int col) {
    assert(row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    return r[col].up;
  }
  
  const float getDown(int row, int col) {
    assert(row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    return r[col].down;
  }
  
  const float getLeft(int row, int col) {
    assert(row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    return r[col].left;
  }
  
  const float getRight(int row, int col) {
    assert(row < grid.size());
    vector<GridPoint> r = grid[row];
    assert(col < r.size());
    return r[col].right;
  }
  
private:
  vector<vector<GridPoint>> grid;
};

#endif /* MAC_h */
