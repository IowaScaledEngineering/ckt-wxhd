#ifndef _FLOAT16_H_
#define _FLOAT16_H_
#include <stdint.h>

typedef uint16_t float16_t;

float16_t F32toF16(float i);
float F16toF32(float16_t in);

#endif

