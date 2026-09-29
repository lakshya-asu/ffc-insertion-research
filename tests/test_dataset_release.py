import hashlib
import json

import pytest

from ffc.dataset_release import REQUIRED, review_release


def manifest_for(tmp_path):
    manifest = {"schema_version": 1, "skill": "pickup", "profile_sha256": "profile-a", "evidence": {}}
    for gate in REQUIRED["pickup"]:
        raw = json.dumps(
            {"gate": gate, "passed": True, "profile_sha256": "profile-a", "review": "Test fixture"}
        ).encode()
        path = tmp_path / f"{gate}.json"
        path.write_bytes(raw)
        manifest["evidence"][gate] = {"path": path.name, "sha256": hashlib.sha256(raw).hexdigest()}
    return manifest


def test_absent_evidence_fails_closed(tmp_path):
    result = review_release({"sha256": "profile-a"}, {"skill": "pickup"}, tmp_path)
    assert not result["evidence_gate_passed"]
    assert len(result["reasons"]) == 6


def test_changed_or_stale_evidence_cannot_pass(tmp_path):
    manifest = manifest_for(tmp_path)
    spec = {"sha256": "profile-a"}
    assert review_release(spec, manifest, tmp_path)["evidence_gate_passed"]
    assert not review_release({"sha256": "profile-b"}, manifest, tmp_path)["evidence_gate_passed"]
    (tmp_path / "contact.json").write_text("{}")
    assert not review_release(spec, manifest, tmp_path)["evidence_gate_passed"]


def test_failed_review_and_path_escape_rejected(tmp_path):
    manifest = manifest_for(tmp_path)
    manifest["evidence"]["contact"]["path"] = "../contact.json"
    assert not review_release({"sha256": "profile-a"}, manifest, tmp_path)["evidence_gate_passed"]
    with pytest.raises(ValueError):
        review_release({"sha256": "profile-a"}, {"skill": "unknown"}, tmp_path)


def test_hash_valid_failed_result_still_rejected(tmp_path):
    manifest = manifest_for(tmp_path)
    raw = json.dumps(
        {"gate": "contact", "passed": False, "profile_sha256": "profile-a", "review": "Failed"}
    ).encode()
    (tmp_path / "contact.json").write_bytes(raw)
    manifest["evidence"]["contact"]["sha256"] = hashlib.sha256(raw).hexdigest()
    assert not review_release({"sha256": "profile-a"}, manifest, tmp_path)["evidence_gate_passed"]
