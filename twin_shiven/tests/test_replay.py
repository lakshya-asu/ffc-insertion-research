import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.export_replay import run_episode, write_usd  # noqa: E402


def test_replay_usd_has_one_cube_per_geom_and_a_frame_per_tick(tmp_path):
    from pxr import Usd, UsdGeom

    ep = run_episode("zero", "tape", 6, seed=3, level="L1", estimator="good")
    assert ep["geom_catalogue"] and all("geoms" in r for r in ep["records"])
    path = write_usd(ep, tmp_path / "replay.usda")
    st = Usd.Stage.Open(str(path))
    cubes = [p for p in st.Traverse() if p.IsA(UsdGeom.Cube)]
    assert len(cubes) == len(ep["geom_catalogue"])
    assert st.GetEndTimeCode() == len(ep["records"]) - 1
    assert st.GetTimeCodesPerSecond() == ep["control_hz"]
    hdr = UsdGeom.Xformable(st.GetPrimAtPath("/Replay/hdr")).GetOrderedXformOps()[0]
    assert hdr.Get(st.GetEndTimeCode())[0] > hdr.Get(0)[0]     # the header moved into the socket
    assert st.GetMetadata("customLayerData")["ffc_twin_replay"]["physics"].startswith("none")
