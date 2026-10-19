#pragma once
/* Template for a new SAD module. Copy and rename. */
#include "sad_types.h"
#include "sad_module.h"

namespace sad {
struct TemplateParams { /* params */ };
struct TemplateState { TemplateParams params; /* state vars */ };
struct TemplateInput { /* inputs */ };
struct TemplateOutput { /* outputs */ };

void template_init(TemplateState& s, const TemplateParams& p);
void template_step(const TemplateInput& u, TemplateState& s, TemplateOutput& y, double dt);
void template_reset(TemplateState& s);
} // namespace sad
