#pragma once
#include <stdbool.h>
#include "drone_types.h"

void web_rc_init(void);
bool web_rc_read(rc_input_t *out);
bool web_rc_is_connected(void);
