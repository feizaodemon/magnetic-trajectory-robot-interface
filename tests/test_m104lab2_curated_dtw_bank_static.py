"""Static checks for the M104-LAB2 curated 8-symbol DTW bank.

No ROS master, Gazebo, GUI, serial, real board, controller_manager, or real-FR3
runtime is started. These tests only read JSON and launch XML.
"""

import json
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
TRACKED_BANK = REPO / "colmag_ros" / "config" / "dtw_banks" / "m104c1_8symbol_mouse_seeded_yflip.json"
IGNORED_OUTPUT_BANK = REPO / "outputs" / "classifier_artifacts" / "dtw_bank" / "m104c1_8symbol_mouse_seeded_yflip.json"
TRACKED_BANK_ARG = "$(find colmag_ros)/config/dtw_banks/m104c1_8symbol_mouse_seeded_yflip.json"
OUTPUT_BANK_FRAGMENT = "outputs/classifier_artifacts/dtw_bank/m104c1_8symbol_mouse_seeded_yflip.json"
TARGET_LABELS = ["1", "2", "3", "V", "O", "X", "A", "C"]

C4_LAUNCH = REPO / "colmag_ros" / "launch" / "m104c4_8symbol_dtw_gazebo_robot_body_demo.launch"
C4_ALIAS = REPO / "colmag_ros" / "launch" / "colmag_8symbol_gazebo_robot_body_demo.launch"
C2C3_LAUNCH = REPO / "colmag_ros" / "launch" / "m104c2c3_8symbol_dtw_recognition_display.launch"
C2C3_ALIAS = REPO / "colmag_ros" / "launch" / "colmag_8symbol_recognition_display.launch"


def _arg_defaults(path):
    root = ET.parse(str(path)).getroot()
    return {node.attrib["name"]: node.attrib.get("default", "") for node in root.findall("arg")}


class CuratedDtwBankTests(unittest.TestCase):
    def setUp(self):
        self.bank = json.loads(TRACKED_BANK.read_text(encoding="utf-8"))

    def test_tracked_bank_exists_and_is_not_under_outputs(self):
        self.assertTrue(TRACKED_BANK.is_file())
        self.assertIn("colmag_ros/config/dtw_banks", TRACKED_BANK.as_posix())
        self.assertNotIn("/outputs/", TRACKED_BANK.as_posix())

    def test_bank_schema_labels_and_template_count(self):
        self.assertEqual(self.bank["schema"], "colmag_dtw_template_bank.v1")
        self.assertEqual(self.bank["model_type"], "dtw_template_bank")
        self.assertEqual(self.bank["labels"], TARGET_LABELS)
        self.assertEqual(len(self.bank["templates"]), 124)
        self.assertEqual({template["label"] for template in self.bank["templates"]}, set(TARGET_LABELS))

    def test_bank_has_no_absolute_machine_path_dependency(self):
        text = TRACKED_BANK.read_text(encoding="utf-8")
        for forbidden in ("/home/", "/tmp/", "/mnt/", "C:\\", "\\\\"):
            self.assertNotIn(forbidden, text)
        for template in self.bank["templates"][:8]:
            self.assertIn("points", template)
            self.assertGreater(len(template["points"]), 0)

    def test_tracked_bank_matches_local_generated_source_bank_when_available(self):
        if not IGNORED_OUTPUT_BANK.is_file():
            self.skipTest("ignored generated source bank is local-only and absent in clean checkouts")
        source = json.loads(IGNORED_OUTPUT_BANK.read_text(encoding="utf-8"))
        self.assertEqual(self.bank["labels"], source["labels"])
        self.assertEqual(len(self.bank["templates"]), len(source["templates"]))
        for curated, generated in zip(self.bank["templates"], source["templates"]):
            self.assertEqual(curated["label"], generated["label"])
            self.assertEqual(curated["points"], generated["points"])

    def test_default_8symbol_launches_use_tracked_bank(self):
        for launch, arg_name in (
            (C4_LAUNCH, "bank_path"),
            (C4_ALIAS, "bank_path"),
            (C2C3_LAUNCH, "dtw_template_bank_path"),
            (C2C3_ALIAS, "dtw_template_bank_path"),
        ):
            args = _arg_defaults(launch)
            self.assertEqual(args[arg_name], TRACKED_BANK_ARG, launch.name)
            self.assertNotIn(OUTPUT_BANK_FRAGMENT, args[arg_name], launch.name)

    def test_generated_outputs_are_not_tracked_or_staged(self):
        tracked_outputs = subprocess.run(
            ["git", "ls-files", "outputs"],
            cwd=str(REPO),
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        staged = subprocess.run(
            ["git", "diff", "--cached", "--name-only"],
            cwd=str(REPO),
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        self.assertEqual(tracked_outputs, "")
        self.assertNotIn("outputs/", staged)
        self.assertNotIn(".joblib", staged)


if __name__ == "__main__":
    unittest.main()
