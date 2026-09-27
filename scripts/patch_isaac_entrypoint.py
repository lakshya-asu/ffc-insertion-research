"""Keep NVIDIA's environment setup, but make Python receive container signals."""

from pathlib import Path

path = Path("/isaac-sim/python.sh")
original = path.read_text()
needle = '    $python_exe "${filtered_args[@]}" $args || error_exit'
replacement = '    exec $python_exe "${filtered_args[@]}" $args'
if replacement not in original:
    if original.count(needle) != 1:
        raise RuntimeError("Pinned Isaac launcher changed; inspect before applying the exec fix")
    path.write_text(original.replace(needle, replacement))
