#ifndef PYRO_COMMON_H
#define PYRO_COMMON_H

#include "pyro_core_def.h"
#include "math.h"

namespace pyro
{

float wrap2pi_f32(float input);

float radps_to_rpm(float radps);

}
#endif