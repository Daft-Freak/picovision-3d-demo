#pragma once

#ifdef PICO_BUILD
// default to fast
#ifndef PICO_INTERP
#define PICO_INTERP 1
#endif

#ifndef PICO_MULTICORE
#define PICO_MULTICORE 1
#endif
#else
// not a pico so these make no sense
#undef PICO_INTERP
#undef PICO_MULTICORE
#endif