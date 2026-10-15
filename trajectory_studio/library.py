"""Trajectory library: versioned recipes organised by experiment category."""

from __future__ import annotations

import re
import shutil
from dataclasses import dataclass
from pathlib import Path

from trajectory_studio.models import CATEGORIES, Trajectory
from trajectory_studio.serialization import export_csv, load_recipe, save_new_revision

DEFAULT_LIBRARY = Path("trajectory_library")
_VERSION_RE = re.compile(r"^(?P<name>.+)\.v(?P<ver>\d+)\.recipe\.json$")


@dataclass
class LibraryEntry:
    name: str
    version: int
    category: str
    path: Path
    description: str
    kind: str
    validation_status: str


def safe_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip()).strip("._")
    if not cleaned:
        raise ValueError("name must contain letters or digits")
    return cleaned


class Library:
    def __init__(self, root: str | Path = DEFAULT_LIBRARY):
        self.root = Path(root)

    def _category_dir(self, category: str) -> Path:
        if category not in CATEGORIES:
            raise ValueError(f"unknown category {category!r}")
        return self.root / safe_name(category)

    def entries(self, search: str = "") -> list[LibraryEntry]:
        """Latest version of each recipe (archived recipes are excluded)."""
        latest: dict[tuple[str, str], LibraryEntry] = {}
        if not self.root.is_dir():
            return []
        for path in sorted(self.root.glob("*/*.recipe.json")):
            if path.parent.name.startswith("_"):
                continue
            m = _VERSION_RE.match(path.name)
            if not m:
                continue
            try:
                traj = load_recipe(path)
            except (ValueError, KeyError, OSError):
                continue
            entry = LibraryEntry(m["name"], int(m["ver"]), traj.category, path, traj.description, traj.kind,
                                 traj.validation_status)
            key = (entry.category, entry.name)
            if key not in latest or entry.version > latest[key].version:
                latest[key] = entry
        needle = search.strip().lower()
        result = [e for e in latest.values()
                  if not needle or needle in e.name.lower() or needle in e.description.lower() or needle in e.kind.lower()]
        return sorted(result, key=lambda e: (CATEGORIES.index(e.category) if e.category in CATEGORIES else 99, e.name))

    def save(self, traj: Trajectory, software_revision: str = "") -> Path:
        traj.name = safe_name(traj.name)
        return save_new_revision(traj, self._category_dir(traj.category), software_revision)

    def load(self, entry: LibraryEntry) -> Trajectory:
        return load_recipe(entry.path)

    def duplicate(self, entry: LibraryEntry, new_name: str) -> Path:
        traj = self.load(entry)
        traj.name = safe_name(new_name)
        traj.version = 1
        traj.validation_status = "DRAFT"
        traj.approval = {"approved": False, "by": "", "date": ""}
        traj.source_csv_sha256 = ""
        traj.experiment.pop("related_run_ids", None)
        return self.save(traj)

    def rename(self, entry: LibraryEntry, new_name: str) -> Path:
        new_path = self.duplicate(entry, new_name)
        self.archive(entry)
        return new_path

    def archive(self, entry: LibraryEntry) -> list[Path]:
        """Move every version of the recipe to _archive (nothing is deleted)."""
        target = self.root / "_archive" / entry.path.parent.name
        target.mkdir(parents=True, exist_ok=True)
        moved = []
        for path in entry.path.parent.glob(f"{entry.name}.v*.recipe.json"):
            moved.append(Path(shutil.move(str(path), str(target / path.name))))
        return moved

    def export_csv(self, traj: Trajectory, path: str | Path) -> Path:
        return export_csv(traj, path)
