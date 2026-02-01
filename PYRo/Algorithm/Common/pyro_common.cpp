#include "pyro_common.h"

namespace pyro
{
float wrap2pi_f32(float input)
{
    return fmodf(input, 2*PI);
}

}