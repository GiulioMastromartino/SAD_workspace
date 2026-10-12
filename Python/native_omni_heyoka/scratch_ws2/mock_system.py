"""Stage-1 mock backend: the pure-Python ``OmniSystem`` contract.

Re-exported from :mod:`omniheyoka.mock` so the scratch checks stay runnable
without a built extension::

    PYTHONPATH=. python scratch_ws2/mock_system.py
"""

from omniheyoka.mock import (  # noqa: F401
    MOCK_SUPPORTED_NOTE,
    MockOmniSystem,
    mock_rw_demo_system,
    mock_rw_pars,
    mock_rw_state,
)

__all__ = [
    "MockOmniSystem",
    "mock_rw_demo_system",
    "mock_rw_pars",
    "mock_rw_state",
    "MOCK_SUPPORTED_NOTE",
]

if __name__ == "__main__":  # pragma: no cover - smoke check
    import numpy as np

    sys_ = mock_rw_demo_system()
    y0 = mock_rw_state()
    pars = mock_rw_pars()
    print("states   :", sys_.state_names())
    print("n_states :", sys_.n_states(), " n_pars:", sys_.n_pars())
    symbolic = sys_.build_symbolic(sys_.state_names())
    print("symbolic :", len(symbolic), "equations")
    acc = sys_.accelerations(y0, pars)
    print("accel    :", len(acc), "->", np.array(acc))
