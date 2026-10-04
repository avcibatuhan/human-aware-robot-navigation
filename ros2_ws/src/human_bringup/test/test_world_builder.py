import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from human_bringup.world_builder import HUMANS, MARKER, build_world, write_world

WORLDS = Path(__file__).resolve().parents[1] / "worlds"


def actor_names(world_file: Path) -> list[str]:
    return [a.get("name") for a in ET.parse(world_file).getroot().iter("actor")]


@pytest.mark.parametrize(
    "enabled",
    [[], ["static_human"], ["walking_human"], ["static_human", "walking_human"]],
)
def test_write_world_inserts_only_enabled_humans(tmp_path, enabled):
    out = write_world(WORLDS, tmp_path / "arena.sdf", enabled)
    assert actor_names(out) == enabled
    assert MARKER not in out.read_text()


def test_world_keeps_static_models(tmp_path):
    out = write_world(WORLDS, tmp_path / "arena.sdf", list(HUMANS))
    models = {m.get("name") for m in ET.parse(out).getroot().iter("model")}
    assert {"wall_north", "wall_south", "wall_east", "wall_west"} <= models
    assert sum(name.startswith("cylinder_") for name in models) == 6


def test_unknown_human_is_rejected():
    with pytest.raises(ValueError, match="unknown human"):
        build_world(f"<world>{MARKER}</world>", {}, ["ghost"])


def test_missing_marker_is_rejected():
    with pytest.raises(ValueError, match="marker"):
        build_world("<world/>", {}, [])
