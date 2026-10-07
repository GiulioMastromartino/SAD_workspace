/*
 ISC License - see native_rw/LICENSE.
 */
%module heyokaMCDynamics

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "heyokaMCDynamics.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "swig_conly_data.i"
%include "std_string.i"
%include "std_vector.i"

namespace std {
    %template(DoubleVector) vector<double>;
}

// IMPORTANT: %import (not %include) for the Basilisk base class, so this
// extension inherits Basilisk's SysModel Python type and AddModelToTask()
// accepts it. See the SDK custom-atmosphere example.
%import "sys_model.i"

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
struct THRArrayOnTimeCmdMsg_C;

%include "heyokaMCDynamics.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
