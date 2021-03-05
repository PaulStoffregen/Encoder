
#include "Encoder.h"

// Yes, all the code is in the header file, to provide the user
// configure options with #define (before they include it), and
// to facilitate some crafty optimizations!

EncoderInternal::Encoder_state_t* EncoderInternal::Encoder::interruptArgs[];

