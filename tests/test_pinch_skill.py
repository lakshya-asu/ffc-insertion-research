from ffc.pinch_skill import PinchObservation, PinchSkill


def sample(t, force=0, travel=0, lift=0):
    return PinchObservation(t, travel, travel, lift, force, force)


def test_no_contact_never_lifts_and_times_out():
    skill = PinchSkill()
    for i in range(7102):
        t = i / 1000
        c = skill.update(t, sample(t, travel=0))
        assert c.lift_m == 0
    assert c.state == "fault" and c.reason == "no_bilateral_contact"


def test_closed_empty_pads_do_not_count_as_a_grasp_even_with_load():
    c = PinchSkill().update(0, sample(0, force=0.2, travel=0.00498))
    assert c.state == "fault" and c.reason == "empty_or_too_thin"
    assert c.lift_m == 0


def test_single_pad_load_does_not_qualify_contact():
    skill = PinchSkill()
    for i in range(100):
        t = i / 1000
        c = skill.update(t, PinchObservation(t, 0, 0, 0, 0.3, 0))
    assert c.state == "close" and c.lift_m == 0


def test_contact_loss_latches_and_freezes_references():
    skill = PinchSkill()
    for i in range(100):
        t = i / 1000
        c = skill.update(t, sample(t, 0.2, lift=skill.lift))
    assert c.state == "lift"
    frozen = skill.update(0.1, sample(0.1, 0, lift=skill.lift))
    assert frozen.state == "fault" and frozen.reason == "contact_lost"
    assert skill.update(0.101, sample(0.101, 0.2)) == frozen


def test_sensor_faults_do_not_issue_more_motion():
    for obs, reason in [
        (sample(-1), "stale_feedback"),
        (sample(0, 2), "bench_force_cap"),
        (sample(0, float("nan")), "nonfinite_feedback"),
    ]:
        c = PinchSkill().update(0, obs)
        assert c.state == "fault" and c.reason == reason
        assert c.lift_m == c.closing_travel_m == 0


def test_completion_is_explicitly_not_grasp_success():
    skill = PinchSkill()
    for i in range(3000):
        t = i / 1000
        c = skill.update(t, sample(t, 0.2, lift=skill.lift))
    assert c.state == "contact_hold_complete"
    assert "camera retention" in c.reason
