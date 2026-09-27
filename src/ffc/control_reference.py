"""Control references that preserve contact preload across skill boundaries."""

import numpy as np


def retain_unchanged_targets(actual, previous_targets, next_targets, indices):
    """Keep an existing position-drive preload when an actuator target is unchanged.

    A contact-loaded actuator deliberately differs from its commanded position.
    Restarting an interpolation at the measured position would remove that load.
    Changed commands still start from the measured position for smooth motion.
    """
    start = np.asarray(actual).copy()
    for index in indices:
        if np.isclose(previous_targets[index], next_targets[index], atol=1e-9, rtol=0):
            start[index] = previous_targets[index]
    return start
