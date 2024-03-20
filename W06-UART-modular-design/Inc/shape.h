#ifndef MODULAR_EXAMPLE_SHAPE_H
#define MODULAR_EXAMPLE_SHAPE_H


#include <stdint.h>


enum {
  SHAPE_SPHERE,
  SHAPE_CUBE,
  SHAPE_CONE
};
  
  
#define true 1
#define false 0


struct shape {
	uint32_t dimension_1;
	uint32_t dimension_2;
	uint32_t type; // from the enum
	float location;
	uint8_t is_initialised;
};


struct shape generate_shape(uint32_t type, uint32_t dimension_1, uint32_t dimension_2);

int initialise_shape(struct shape *_shape, uint32_t type, uint32_t dimension_1, uint32_t dimension_2);

// Assumes you have a valid char buffer at address string
void shape_string(struct shape *_shape, uint8_t *string);

#endif //MODULAR_EXAMPLE_SHAPE_H
