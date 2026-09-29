from ffc.insertion_skill import FeedObservation, InsertionFeed


def obs(t, x=0, fixture=0, pad=0.15):
    return FeedObservation(t, x, pad, pad, fixture)


def test_completed_motion_does_not_mean_seated():
    skill = InsertionFeed(distance_m=0.0001)
    for i in range(30):
        command = skill.update(i * 0.005, obs(i * 0.005, skill.target))
    assert command.state == "travel_complete_unverified"


def test_obstruction_retracts_and_latches():
    skill = InsertionFeed()
    skill.update(0, obs(0))
    command = skill.update(0.005, obs(0.005, 0.001, 0.3))
    assert command.state == "retract" and command.target_m < 0.001
    for i in range(2, 120):
        command = skill.update(i * 0.005, obs(i * 0.005, skill.target))
    assert command.state == "retracted"
    assert abs(command.target_m - 0.0005) < 1e-12


def test_stale_feedback_stops_without_blind_retract():
    skill = InsertionFeed()
    skill.update(0, obs(0))
    command = skill.update(0.1, obs(0, 0.0002))
    assert command.state == "stopped" and command.target_m == 0.0002
    assert skill.update(0.105, obs(0.105)).state == "stopped"


def test_grasp_loss_and_persistent_load_stop():
    skill = InsertionFeed()
    assert skill.update(0, obs(0, pad=0)).state == "stopped"
    skill = InsertionFeed()
    assert skill.update(0, obs(0, fixture=0.6)).reason == "persistent_overload"


def test_feed_stall_is_not_success():
    skill = InsertionFeed()
    for i in range(45):
        command = skill.update(i * 0.005, obs(i * 0.005))
        if command.state != "advance":
            break
    assert command.reason == "feed_tracking_error"
