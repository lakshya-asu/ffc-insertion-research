"""Write the MuJoCo and USD scenes plus the spec JSON to out/."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.mjcf import build_mjcf  # noqa: E402
from ffc_twin.spec import DEFAULT  # noqa: E402
from ffc_twin.usd import build_usd  # noqa: E402

from ffc_twin.spec import BOARDS  # noqa: E402

out = Path(__file__).resolve().parents[1] / "out"
out.mkdir(exist_ok=True)
for board, spec in BOARDS.items():
    stem = "ffc_twin" if board == "pi4" else f"ffc_twin_{board}"
    (out / f"{stem}.xml").write_text(build_mjcf(spec))
    (out / f"{stem}_spec.json").write_text(spec.to_json())
    build_usd(out / f"{stem}.usda", spec)
    print(board, "placeholders:", len(spec.placeholders()))
print("wrote", sorted(p.name for p in out.iterdir() if p.name.startswith("ffc_twin")))
for p in DEFAULT.placeholders():
    print("  -", p)
