#pragma once

#include "fjalar.h"
#include "flight_state.h"

void set_pyro(fjalar_t *fjalar, int pyro, bool state);

void init_actuation(fjalar_t *fjalar);