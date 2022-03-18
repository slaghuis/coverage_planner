#pragma once

#include <vector>
#include <utility>    // std::pair std::make_pair
#include <stdlib.h>   // abs
#include <cmath>      // pow, sqrt, atan
#include <iostream>   // << Overloading
#include <limits>     // std::numeric_limits<double>::max()
#include "geometry_msgs/msg/polygon.hpp"

struct Vertex {
  double x, y;
};

class Edge {
  public:    
    Edge(Vertex *a, Vertex *b);
    std::pair<double, double> line();
    double length();
    double inclination();
    double distance_to(Vertex *a);
    std::pair<double, double> point_from_start(double d);
    bool intersect(double m, double c, Vertex *a);
    bool on_segment(Vertex *a);
    friend std::ostream &operator<<( std::ostream &output, const Edge &E ) {
      output << "[" << E.v_start->x << ", " << E.v_start->y << "] - [" << E.v_end->x << ", " << E.v_end->y << "]";
      return output;
    }    
  private:
    Vertex *v_start;
    Vertex *v_end;
    double v_length, v_slope, v_intercept;
};  

