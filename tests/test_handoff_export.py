import json, tempfile, unittest, zipfile, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import export_identification_handoff as ex
from tests.test_audit_readiness import make_rows, write_run


class HandoffTests(unittest.TestCase):
    def test_zip_matches_selected_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = write_run(tmp, make_rows())
            zip_path = ex.build(run, Path(tmp) / "out")
            self.assertIn(run.stem, zip_path.name)
            with zipfile.ZipFile(zip_path) as z:
                names = z.namelist()
                self.assertIn(f"raw/{run.name}", names)
                self.assertIn("run_report/RUN_AUDIT.json", names)
                audit = json.loads(z.read("run_report/RUN_AUDIT.json"))
                self.assertEqual(audit["source_sha256"], ex.sha256(run))
                self.assertEqual(z.read(f"raw/{run.name}"), run.read_bytes())
        # report directory created under repo reports/identification_runs for run.csv
        import shutil
        shutil.rmtree(ex.ROOT / "reports" / "identification_runs" / run.stem, ignore_errors=True)

    def test_missing_run_is_an_error(self):
        with self.assertRaises(SystemExit):
            ex.build(Path("no_such_run.csv"), Path(tempfile.gettempdir()))


if __name__ == "__main__":
    unittest.main()
