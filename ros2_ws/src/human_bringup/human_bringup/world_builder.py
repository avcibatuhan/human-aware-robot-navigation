"""Build the arena world with an optional set of human actors.

The world file carries a ``<!-- HUMANS -->`` marker; each requested actor
snippet from ``worlds/humans/`` is inserted there. Kept free of ROS imports so
it can be unit-tested on its own.
"""

from pathlib import Path

MARKER = "<!-- HUMANS -->"
HUMANS = ("static_human", "walking_human")


def build_world(world_text: str, snippets: dict[str, str], enabled: list[str]) -> str:
    """Return ``world_text`` with the enabled actor snippets in place of the marker."""
    if world_text.count(MARKER) != 1:
        raise ValueError(f"world must contain exactly one '{MARKER}' marker")
    unknown = [name for name in enabled if name not in snippets]
    if unknown:
        raise ValueError(f"unknown human(s): {', '.join(unknown)}")
    actors = "\n".join(snippets[name].rstrip() for name in enabled)
    return world_text.replace(MARKER, actors.strip() if actors else "")


def write_world(worlds_dir: Path, output: Path, enabled: list[str]) -> Path:
    """Read ``arena.sdf`` and the actor snippets from ``worlds_dir``, write the result."""
    snippets = {name: (worlds_dir / "humans" / f"{name}.sdf").read_text() for name in HUMANS}
    world = build_world((worlds_dir / "arena.sdf").read_text(), snippets, enabled)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(world)
    return output
