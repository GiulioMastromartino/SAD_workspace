#include "sad_timing.h"

namespace sad {

thread_local TimingAccum g_timing;

const char* timing_slot_name(int slot) {
    switch (slot) {
        case TSun:            return "sun_step";
        case TSunPointQuat:   return "  sun_point_quat";
        case TStarTracker:    return "startracker_step";
        case TMission:        return "mission_step";
        case TQuatPD:         return "quatpd_step";
        case TAlloc:          return "thruster_allocate";
        case TAllocBuildA:    return "  alloc_build_A";
        case TAllocSVD:       return "  alloc_jacobiSVD";
        case TAllocClamp:     return "  alloc_clamp_scale";
        case TAllocSolve:     return "  alloc_solve";
        case TAllocInit:      return "allocator_init";
        case TThrusterStep:   return "thruster_step";
        case TThrNorm:        return "  thr_dir_normalize";
        case TDynamics:       return "dynamics_step";
        case TDynamicsAlloc:  return "  dyn_vector_allocs";
        case TRhsTotal:       return "  rhs_total(4 stages)";
        case TRhsDcm:         return "    rhs_quat_to_dcm";
        case TRhsGravityGrad: return "    rhs_gravity_grad";
        case TRhsQuatDeriv:   return "    rhs_quat_deriv";
        case TKpi:            return "kpi_bookkeeping";
        case TDispersion:     return "dispersion";
        case TPerRunSetup:    return "per_run_setup";
        case TLoopOverhead:   return "loop_iteration_total";
        default:              return "?";
    }
}

} // namespace sad
