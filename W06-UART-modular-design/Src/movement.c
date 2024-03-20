#include "movement.h"


// These are internal variables (no one outside this file scope needs to know)
const float SHAPE_SPHERE_MULTIPLIER = 5;
const float SHAPE_CUBE_MULTIPLIER = 1.5;
const float SHAPE_CONE_MULTIPLIER = 0.5;


float roll_unit_distance(struct shape *_shape) {

  if (_shape->type == SHAPE_SPHERE) {
    // calculations show that one unit of push moves the sphere 5 x the diameter
    return SHAPE_SPHERE_MULTIPLIER * (float)(_shape->dimension_1);
  }
  else if (_shape->type == SHAPE_CUBE) {
    // calculations that one unit of push moves the cube 1.5 x the side length
    return SHAPE_CUBE_MULTIPLIER * (float)(_shape->dimension_1);
  }
  else if (_shape->type == SHAPE_CONE) {
    // calculations that one unit of push moves the cube 1.5 x the side length
    return SHAPE_CONE_MULTIPLIER * (float)(_shape->dimension_1);
  }
}


void roll_shape(struct shape *_shape, float push_strength) {
  _shape->location += push_strength * roll_unit_distance(_shape);
}


void slide_shape(struct shape *_shape, float push_strength) {
  _shape->location += push_strength;
}
