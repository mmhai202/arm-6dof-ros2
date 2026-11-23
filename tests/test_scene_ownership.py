"""Exercise ownership across real processes without a running ROS graph."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

try:
    from arm_moveit_config.scene_ownership import SceneOwnership
except ImportError as exc:
    raise unittest.SkipTest("Activate Conda and source install/setup.bash") from exc


class SceneOwnershipTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "scene.lock"
        self.owner = SceneOwnership(self.path)
        self.addCleanup(self.owner.close)
        self.assertTrue(self.owner.claim())

    def contender(self):
        return subprocess.check_output([
            sys.executable, "-c",
            "import os, sys; from arm_moveit_config.scene_ownership import SceneOwnership; "
            "owner = SceneOwnership(sys.argv[1]); print(owner.claim(), flush=True); os._exit(0)",
            str(self.path),
        ], text=True, timeout=10).strip()

    def test_other_process_cannot_claim_an_active_task(self):
        self.assertEqual(self.contender(), "False")
        self.assertTrue(self.owner.resume())

    def test_idle_owner_stays_retired_after_replacement_exits(self):
        self.owner.release()
        self.assertEqual(self.contender(), "True")
        self.assertFalse(self.owner.resume())
        self.assertTrue(self.owner.superseded)
        self.assertFalse(self.owner.resume())

    def test_process_exit_releases_the_lock_without_deleting_it(self):
        self.owner.close()
        self.assertTrue(self.path.is_file())
        self.assertEqual(self.contender(), "True")
        replacement = SceneOwnership(self.path)
        self.addCleanup(replacement.close)
        self.assertTrue(replacement.claim())
