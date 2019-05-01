
#include "Encoder.h"

// Yes, all the code is in the header file, to provide the user
// configure options with #define (before they include it), and
// to facilitate some crafty optimizations!

#ifndef ENCODER_USE_FUNCTIONAL_INTERRUPTS
Encoder_internal_state_t * Encoder::interruptArgs[];
#endif


