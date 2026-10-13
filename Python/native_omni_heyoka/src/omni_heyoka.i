/*
  ISC License

  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

  Permission to use, copy, modify, and/or distribute this software for any
  purpose with or without fee is hereby granted, provided that the above
  copyright notice and this permission notice appear in all copies.

  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
  ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE SOFTWARE OR THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
%module omni_heyoka

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "omni/spacecraft.hpp"
   #include "omni/vec_integrator.hpp"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "swig_conly_data.i"
%include "std_string.i"
%include "std_vector.i"

// Stock integrator base: SWIG must parse the REAL declaration (not just the
// C %{ %} include) so OmniVecIntegrator's descriptor carries the
// derived->base cast that setIntegrator()'s ConvertPtr needs cross-module.
// (Raw-header parsing only; no wrappers for stock internals are emitted.)
%include "simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h"

namespace std {
    %template(DoubleVector) vector<double>;
}

// IMPORTANT: %import (not %include) for the Basilisk base class, so this
// extension inherits Basilisk's SysModel Python type and AddModelToTask()
// accepts it.  Same reasoning as native_rw / the SDK custom-atmosphere example.
%import "sys_model.i"

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
struct ArrayMotorTorqueMsg_C;
%include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
struct RWSpeedMsg_C;
%include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
struct RWConfigLogMsg_C;
%template(RWConfigLogMsgVector) std::vector<Message<RWConfigLogMsgPayload>>;
// WP25-B2.1: RW-array config hijack surface (guard-only ReadFunctor on the
// SysModel; a linked message is loudly rejected, never read).
%include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
struct RWArrayConfigMsg_C;
%include "architecture/msgPayloadDefC/FuelTankMsgPayload.h"
struct FuelTankMsg_C;
%include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
struct THRArrayOnTimeCmdMsg_C;
%include "architecture/msgPayloadDefC/THRArrayCmdForceMsgPayload.h"
struct THRArrayCmdForceMsg_C;
%include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"
// One Message<THROutputMsgPayload> per thruster (WP3 thrusterOutMsgs);
// the vector template exposes stock-style outs[i].recorder() indexing.
// (If host SWIG rejects this line, drop it: the member still exists in C++.)
%template(THROutputMsgVector) std::vector<Message<THROutputMsgPayload>>;
%include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"
struct AtmoPropsMsg_C;
%include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
struct EclipseMsg_C;
%include "architecture/msgPayloadDefC/MagneticFieldMsgPayload.h"
struct MagneticFieldMsg_C;
%include "architecture/msgPayloadDefC/EarthRadiationMsgPayload.h"
struct EarthRadiationMsg_C;
%include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
struct SpicePlanetStateMsg_C;
%include "architecture/msgPayloadDefC/WindMsgPayload.h"
struct WindMsg_C;

%include "omni/spacecraft.hpp"
%include "omni/vec_integrator.hpp"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
