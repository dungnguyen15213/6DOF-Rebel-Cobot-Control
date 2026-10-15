"""Hardware-independent tests for the Trajectory Studio (authoring/validation only)."""
import csv
import hashlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from trajectory_studio import experiment_planner as ep
from trajectory_studio import generators as gen
from trajectory_studio import validators
from trajectory_studio.library import Library
from trajectory_studio.models import Trajectory, Waypoint
from trajectory_studio.serialization import (CsvImportError, export_csv, import_csv, load_recipe,
                                             save_new_revision, save_recipe)

FIXTURE = ROOT / "Pathways" / "trajectory_export.csv"
START = [0.0, -40.0, 100.0, 0.0, 70.0, 0.0]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


class CsvTests(unittest.TestCase):
    def test_import_existing_and_roundtrip_byte_identical(self):
        traj, _ = import_csv(FIXTURE)
        self.assertEqual(len(traj.waypoints), 28)
        self.assertEqual(traj.waypoints[0].phase, "PREPARATION")
        self.assertEqual(traj.velocity_percent, 50.0)
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "x.csv"
            export_csv(traj, out)
            self.assertEqual(sha(out), sha(FIXTURE))

    def test_exported_header(self):
        traj, _ = import_csv(FIXTURE)
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "x.csv"
            export_csv(traj, out)
            with open(out, newline="") as fh:
                header = next(csv.reader(fh))
        self.assertEqual(header, ["Phase", "Step", "J1 (deg)", "J2 (deg)", "J3 (deg)", "J4 (deg)",
                                  "J5 (deg)", "J6 (deg)", "Velocity (%)"])

    def test_bad_csv_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.csv"
            p.write_text("Phase,Step,J1 (deg)\nTASK,1,abc\n")
            with self.assertRaises(CsvImportError):
                import_csv(p)
            p.write_text("")
            with self.assertRaises(CsvImportError):
                import_csv(p)


class GeneratorTests(unittest.TestCase):
    def test_single_joint_only_selected_moves(self):
        t = gen.single_joint_sweep("a", 1, START, -20.0, velocity=10)
        self.assertEqual(t.moving_joints(), [1])
        for w in t.waypoints:
            for j in (0, 2, 3, 4, 5):
                self.assertEqual(w.joints[j], START[j])
        self.assertTrue(validators.validate(t).exportable)

    def test_bidirectional_structure(self):
        t = gen.bidirectional_sweep("b", 0, START, 10.0, -10.0, segments=2, velocity=10)
        self.assertEqual(t.waypoints[0].joints, START)
        self.assertEqual(t.waypoints[-1].joints, START)
        self.assertEqual(t.joint_range(0), (-10.0, 10.0))
        self.assertEqual(t.moving_joints(), [0])

    def test_missing_values_not_defaulted(self):
        with self.assertRaises(gen.IncompleteTemplate):
            gen.single_joint_sweep("a", 0, START, None)
        with self.assertRaises(gen.IncompleteTemplate):
            gen.single_joint_sweep("a", 0, None, 5.0)

    def test_test_b_templates_blocked_until_completed(self):
        for joint in (3, 5):
            t = gen.test_b_template(joint)
            rep = validators.validate(t)
            self.assertFalse(rep.exportable)
            with self.assertRaises(gen.IncompleteTemplate):
                gen.complete_template(t)
            done = gen.complete_template(t, start_pose=START, positive_deg=5.0, negative_deg=-5.0, velocity=10)
            self.assertTrue(validators.validate(done).exportable)

    def test_stationary_not_exportable(self):
        rep = validators.validate(gen.stationary_hold("hold"))
        self.assertFalse(rep.exportable)

    def test_multi_speed(self):
        base = gen.single_joint_sweep("a", 0, START, 10.0, velocity=10)
        v = gen.multi_speed_variants(base, [10, 20])
        self.assertEqual([x.velocity_percent for x in v], [10, 20])
        self.assertEqual(v[0].waypoints[1].joints, v[1].waypoints[1].joints)


class ValidatorTests(unittest.TestCase):
    def test_limits_not_verified_and_violation_blocks(self):
        t = gen.single_joint_sweep("a", 0, START, 10.0, velocity=10)
        rep = validators.validate(t)
        self.assertTrue(rep.has("JOINT_LIMITS"))
        self.assertIn("NOT VERIFIED", rep.to_text())
        t.joint_limits_deg = [[-5.0, 5.0]] + [[-360.0, 360.0]] * 5
        t.limits_source = "unit test"
        self.assertFalse(validators.validate(t).exportable)

    def test_bad_values(self):
        t = gen.single_joint_sweep("a", 0, START, 10.0, velocity=10)
        t.waypoints[1].joints[0] = float("nan")
        self.assertFalse(validators.validate(t).exportable)
        t = gen.single_joint_sweep("a", 0, START, 10.0, velocity=10)
        t.velocity_percent = 500
        self.assertFalse(validators.validate(t).exportable)
        self.assertFalse(validators.validate(Trajectory(name="e")).exportable)

    def test_locked_joint_moving_flagged(self):
        t = gen.single_joint_sweep("a", 0, START, 10.0, velocity=10)
        t.locked_joints[0] = True
        self.assertFalse(validators.validate(t).exportable)


class EditAndLibraryTests(unittest.TestCase):
    def test_edit_ops(self):
        t, _ = import_csv(FIXTURE)
        t.insert_waypoint(3, Waypoint("TASK", [1, 2, 3, 4, 5, 6]))
        self.assertEqual(len(t.waypoints), 29)
        t.move_waypoint(3, 5)
        self.assertEqual(t.waypoints[5].joints, [1, 2, 3, 4, 5, 6])
        t.delete_waypoints([5])
        self.assertEqual(len(t.waypoints), 28)
        t.duplicate_waypoints([2])
        self.assertEqual(len(t.waypoints), 29)
        self.assertEqual(t.waypoints[0].phase, "PREPARATION")
        self.assertTrue(all(w.phase == "TASK" for w in t.waypoints[1:]))

    def test_recipe_versioning_and_metadata(self):
        t = gen.single_joint_sweep("rec", 0, START, 10.0, velocity=10)
        t.experiment = {"payload_g": 0, "condition": "NORMAL"}
        with tempfile.TemporaryDirectory() as d:
            p1 = save_new_revision(t, d)
            p2 = save_new_revision(t, d)
            self.assertNotEqual(p1, p2)
            self.assertTrue(p1.exists() and p2.exists())
            back = load_recipe(p2)
            self.assertEqual(back.experiment, t.experiment)
            self.assertEqual([w.joints for w in back.waypoints], [w.joints for w in t.waypoints])

    def test_library_archive_never_deletes(self):
        t = gen.single_joint_sweep("lib", 0, START, 10.0, velocity=10)
        with tempfile.TemporaryDirectory() as d:
            lib = Library(d)
            lib.save(t)
            e = lib.entries()[0]
            lib.duplicate(e, "lib_copy")
            self.assertEqual(len(lib.entries()), 2)
            moved = lib.archive(e)
            self.assertTrue(all(Path(m).exists() for m in moved))
            self.assertEqual(len(lib.entries()), 1)


class CampaignTests(unittest.TestCase):
    def test_default_plan_15_recipes_no_angles(self):
        c = ep.Campaign.default_plan("ID_NORMAL_001", 2)
        self.assertEqual(len(c.recipes), 15)
        self.assertEqual(len(c.manifest()["recipes"]), 15)
        for r in c.recipes:
            self.assertEqual(r.waypoints, [])
            self.assertFalse(validators.validate(r).exportable)
        cov = ep.coverage(c)
        self.assertTrue(cov)

    def test_link_and_compare_only_read_runs(self):
        t = gen.single_joint_sweep("rec", 0, START, 10.0, velocity=10)
        with tempfile.TemporaryDirectory() as d:
            rp = save_recipe(t, Path(d) / "r.recipe.json")
            run = Path(d) / "run.csv"
            with open(run, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp"] + [f"q{i}" for i in range(1, 7)] + [f"current{i}" for i in range(1, 7)])
                for k in range(5):
                    w.writerow([k] + [START[0] + k * 2] + START[1:] + [0.1] * 6)
            before = sha(run)
            ep.link_run(rp, run, Path(d) / "links.json")
            res = ep.compare_planned_recorded(t, run)
            self.assertTrue(res)
            self.assertEqual(sha(run), before)
            self.assertTrue((Path(d) / "links.json").exists())


class IsolationTests(unittest.TestCase):
    def test_engine_does_not_import_hardware(self):
        for p in (ROOT / "trajectory_studio").glob("*.py"):
            src = p.read_text(encoding="utf-8")
            for banned in ("import cri_lib", "from cri_lib", "import hardware", "from hardware",
                           "from core", "import core", "ControlManager", "execute_path"):
                self.assertNotIn(banned, src, f"{p.name} references {banned}")


class WindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def make(self, d):
        from gui.trajectory_studio_window import TrajectoryStudioWindow
        return TrajectoryStudioWindow(library_root=d, pathways_dir=Path(d) / "pw")

    def test_load_undo_redo(self):
        with tempfile.TemporaryDirectory() as d:
            w = self.make(d)
            self.assertTrue(w.import_csv_path(FIXTURE))
            m = w.model
            n = len(m.traj.waypoints)
            m.insert_after(2)
            self.assertEqual(len(m.traj.waypoints), n + 1)
            m.undo_stack.undo()
            self.assertEqual(len(m.traj.waypoints), n)
            m.undo_stack.redo()
            self.assertEqual(len(m.traj.waypoints), n + 1)
            m.delete_rows([3])
            m.undo_stack.undo()
            self.assertEqual(len(m.traj.waypoints), n + 1)
            w.close()

    def test_export_blocked_for_template_and_send_only_emits(self):
        with tempfile.TemporaryDirectory() as d:
            w = self.make(d)
            w.model.set_trajectory(gen.test_b_template(3))
            self.assertIsNone(w.export_to(Path(d) / "x.csv"))
            self.assertFalse((Path(d) / "x.csv").exists())
            w.import_csv_path(FIXTURE)
            out = w.export_to(Path(d) / "ok.csv")
            self.assertIsNotNone(out)
            self.assertEqual(sha(out), sha(FIXTURE))
            w.close()

    def test_campaign_panel(self):
        with tempfile.TemporaryDirectory() as d:
            w = self.make(d)
            w.create_campaign()
            self.assertEqual(len(w.campaign.recipes), 15)
            w.close()


if __name__ == "__main__":
    unittest.main()

