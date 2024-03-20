#include "shape.h"

#include <stdio.h>


struct shape generate_shape(uint32_t type, uint32_t dimension_1, uint32_t dimension_2) {

  struct shape new_shape;

  new_shape.type = type;
  new_shape.dimension_1 = dimension_1;
  new_shape.dimension_2 = dimension_2;
  new_shape.location = 0.;
  new_shape.is_initialised = true;

  return new_shape;
}


// returns true for success, false for failure
int initialise_shape(struct shape *_shape, uint32_t type, uint32_t dimension_1, uint32_t dimension_2) {

  if (dimension_1 <= 0 || dimension_2 <= 0)
    return false;

  _shape->type = type;
  _shape->dimension_1 = dimension_1;
  _shape->dimension_2 = dimension_2;
  _shape->location = 0.;
  _shape->is_initialised = true;

  return true;
}


void shape_string(struct shape *_shape, uint8_t *string) {
  
  switch(_shape->type) {
    case SHAPE_SPHERE:
      sprintf((int8_t*)string, "Shape is a SPHERE, radius is %lu, location is %.2f \r\n", _shape->dimension_1, _shape->location);
      break;
    case SHAPE_CONE:
      sprintf((int8_t*)string, "Shape is a CONE, radius is %lu, height is %lu, location is %.2f \r\n", _shape->dimension_1, _shape->dimension_2, _shape->location);
      break;
    case SHAPE_CUBE:
      sprintf((int8_t*)string, "Shape is a CUBE, side length is %lu, location is %.2f \r\n", _shape->dimension_1, _shape->location);
      break;
    default:
      sprintf((int8_t*)string, "Shape is a UNKNOWN, type is %lu, location is %.2f \r\n", _shape->type, _shape->location);
      break;
    
  }
}


/*
void PrintSphere(struct shape *_shape, char *string) {
  sprintf(&string[0], "Shape is a SPHERE, radius is %d, location is %.2f \r\n", _shape->dimension_1, _shape->location);    
}

void PrintCone(struct shape *_shape, char *string) {
  sprintf(&string[0], "Shape is a CONE, radius is %d, height is %d, location is %.2f \r\n", _shape->dimension_1, _shape->dimension_2, _shape->location);    
}

void PrintCube(struct shape *_shape, char *string) {
  sprintf(&string[0], "Shape is a CUBE, side length is %d, location is %.2f \r\n", _shape->dimension_1, _shape->location);      
}

*/
