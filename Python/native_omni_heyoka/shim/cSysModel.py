"""Top-level ``cSysModel`` alias for SWIG-generated Basilisk extensions. ISC License - see native_rw/LICENSE.

SWIG emits a bare ``import cSysModel`` for ``%import "sys_model.i"``
(the module name declared by the SDK's ``sys_model.i``), but the built
artifact lives at ``Basilisk.architecture.cSysModel``. This shim makes the
bare import resolve to the genuine Basilisk class, so extension wrappers
inherit the scheduler-visible ``SysModel`` type. Re-exported, not copied:
``cSysModel.SysModel is Basilisk.architecture.cSysModel.SysModel``.
"""
from Basilisk.architecture.cSysModel import SysModel

__all__ = ["SysModel"]
