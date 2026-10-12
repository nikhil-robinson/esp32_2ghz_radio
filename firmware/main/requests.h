#pragma once

#include "link.h"

/* Execute one host request and send its reply frames. */
void requests_handle(const link_request_t *request);
