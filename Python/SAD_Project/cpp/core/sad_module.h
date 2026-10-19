#pragma once
// Common module lifecycle contract. Each module defines its own typed input/output/state structs
// in its header, but follows this pattern. Simulink S-functions and pybind shims call these.

#include "sad_types.h"
#include <cstddef>

namespace sad {

// Generic step signature helper macros are not used; modules declare:
//   void <mod>_init(<mod>_state_t&, const <mod>_params_t&);
//   void <mod>_step(const <mod>_input_t&, <mod>_state_t&, <mod>_output_t&, double dt);
//   void <mod>_reset(<mod>_state_t&);

// A minimal parameter/state base for modules that need nothing extra.
struct EmptyParams {};
struct EmptyState {};
struct EmptyInput {};
struct EmptyOutput {};

} // namespace sad
