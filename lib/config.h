#pragma once

#ifdef PICO_BUILD
// default to fast
#ifndef THR3E_PICO_INTERP
#define THR3E_PICO_INTERP 1
#endif

#ifndef THR3E_PICO_MULTICORE
#define THR3E_PICO_MULTICORE 1
#endif
#else
// not a pico so these make no sense
#undef THR3E_PICO_INTERP
#undef THR3E_PICO_MULTICORE
#endif