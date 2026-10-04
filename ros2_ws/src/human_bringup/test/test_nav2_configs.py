from pathlib import Path

import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config"


def load(name):
    return yaml.safe_load((CONFIG / name).read_text())


@pytest.fixture
def baseline():
    return load("nav2_baseline.yaml")


@pytest.fixture
def social():
    return load("nav2_social.yaml")


def local_costmap(config):
    return config["local_costmap"]["local_costmap"]["ros__parameters"]


def test_the_only_difference_is_the_social_layer(baseline, social):
    params = local_costmap(social)
    assert params["plugins"][-1] == "social_layer"
    params["plugins"].remove("social_layer")
    del params["social_layer"]
    assert social == baseline


def test_baseline_has_no_social_layer(baseline):
    params = local_costmap(baseline)
    assert "social_layer" not in params["plugins"]
    assert "social_layer" not in params


def test_social_layer_parameters(social):
    assert local_costmap(social)["social_layer"] == {
        "plugin": "social_costmap_layer::SocialLayer",
        "enabled": True,
        "humans_topic": "/tracked_humans_map",
        "social_radius": 1.2,
        "lethal_radius": 0.25,
        "sigma": 0.45,
        "max_human_age": 2.0,
        "peak_cost": 220,
    }


@pytest.mark.parametrize("name", ["nav2_baseline.yaml", "nav2_social.yaml"])
def test_controller_parameters(name):
    controller = load(name)["controller_server"]["ros__parameters"]
    assert controller["controller_frequency"] == 10.0
    assert controller["FollowPath"]["plugin"] == "dwb_core::DWBLocalPlanner"
    assert controller["FollowPath"]["max_vel_x"] == 0.3
    assert controller["FollowPath"]["max_vel_theta"] == 1.0
