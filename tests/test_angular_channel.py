from __future__ import annotations

import numpy as np

from hybrid_controller.controllers.angular_channel import evaluate_angular_channel


def test_centerline_obstacle_selects_the_more_reachable_turn_side():
    decision = evaluate_angular_channel(
        np.array([0.0, 0.0, 0.0, 0.2, 0.0]),
        {"x": 2.0, "y": 0.0, "radius": 0.1},
        previous_omega_command=0.15,
        max_delta_omega=0.15,
        minimum_turn_rate=0.1,
        trigger_clearance=0.0,
    )
    assert decision.forward_channel
    assert decision.selected_side == "left"
    assert decision.left_margin > decision.right_margin
    assert decision.trigger is False


def test_left_bearing_requires_the_right_turn_channel():
    decision = evaluate_angular_channel(
        np.array([0.0, 0.0, 0.0, 0.2, 0.0]),
        {"x": 0.2, "y": 0.15, "radius": 0.1},
        previous_omega_command=0.8,
        max_delta_omega=0.05,
        minimum_turn_rate=0.5,
        trigger_clearance=0.0,
    )
    assert decision.selected_side == "right"
    assert decision.right_margin < 0.0
    assert decision.trigger


def test_outside_forward_channel_does_not_trigger():
    decision = evaluate_angular_channel(
        np.array([0.0, 0.0, 0.0, 0.2, 0.0]),
        {"x": 0.0, "y": 1.0, "radius": 0.1},
        previous_omega_command=0.0,
        channel_half_angle=0.5,
        trigger_clearance=1.0,
    )
    assert not decision.forward_channel
    assert not decision.trigger
