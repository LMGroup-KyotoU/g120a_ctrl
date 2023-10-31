#include "utils.h"

double clamp(double value, double min, double max) {
  if(value > max) 
    return max;
  else if(value < min) 
    return min;
  else
    return value;
}