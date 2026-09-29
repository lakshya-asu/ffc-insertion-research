"""Write the MuJoCo and USD scenes plus the spec JSON to out/."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.mjcf import build_mjcf  # noqa: E402
from ffc_twin.spec import DEFAULT  # noqa: E402
from ffc_twin.usd import build_usd  # noqa: E402

out = Path(__file__).resolve().parents[1] / "out"
out.mkdir(exist_ok=True)
(out / "ffc_twin.xml").write_text(build_mjcf())
(out / "ffc_twin_spec.json").write_text(DEFAULT.to_json())
build_usd(out / "ffc_twin.usda")
print("wrote", sorted(p.name for p in out.iterdir()))
print("placeholders:")
for p in DEFAULT.placeholders():
    print("  -", p)
