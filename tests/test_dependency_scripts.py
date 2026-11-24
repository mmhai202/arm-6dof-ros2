"""Exercise installer/launcher shell flows without downloading ROS or changing Conda."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[1]


class DependencyScriptsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "workspace with spaces"
        self.root.mkdir()
        for folder in ("scripts",):
            shutil.copytree(REPO / folder, self.root / folder)
        self.prefix = self.root / ".conda"
        self.base = Path(self.temp.name) / "base"
        self.log = Path(self.temp.name) / "calls.log"
        self.env = dict(os.environ)
        self.env.update(
            ARM_CONDA_PREFIX=str(self.prefix),
            CONDA_EXE=str(self.base / "bin/conda"),
            MOCK_BASE=str(self.base),
            MOCK_LOG=str(self.log),
            PATH="/opt/ros/humble/bin:/usr/bin:/bin",
            PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages",
            AMENT_PREFIX_PATH=f"/opt/ros/humble:{self.root}/install/old_package",
            CMAKE_PREFIX_PATH="/opt/ros/humble",
            LD_LIBRARY_PATH="/opt/ros/humble/lib:/unrelated/lib",
            ROS_DOMAIN_ID="42",
        )
        self.write_executable(self.base / "bin/conda", """#!/usr/bin/env bash
set -eu
if [[ "$1" == info ]]; then echo "$MOCK_BASE"; exit; fi
printf '%s\\n' "$*" >> "$MOCK_LOG"
[[ "${MOCK_INSTALL_FAIL:-0}" == 0 ]] || exit 17
while [[ $# -gt 0 ]]; do
  if [[ "$1" == --prefix ]]; then mkdir -p "$2/conda-meta"; break; fi
  shift
done
""")
        profile = self.base / "etc/profile.d/conda.sh"
        profile.parent.mkdir(parents=True)
        profile.write_text("""conda() {
  if [[ "$1" == activate ]]; then
    # Simulate an activation hook that is incompatible with nounset.
    local ignored="$MOCK_UNSET_VARIABLE"
    export CONDA_PREFIX="$2" PATH="$2/bin:$PATH"
    if [[ -n "${MOCK_SHADOW_BIN:-}" ]]; then
      export PATH="$MOCK_SHADOW_BIN:$PATH"
    fi
    if [[ "${MOCK_NO_COMPILER_VARS:-0}" == 1 ]]; then
      unset CC CXX
    else
      export CC="${MOCK_CC:-$2/bin/cc}" CXX="$2/bin/c++"
    fi
    export PYTHONPATH="$2/modules${PYTHONPATH:+:$PYTHONPATH}"
    printf 'activate %s\\n' "$2" >> "$MOCK_LOG"
  else
    echo 'Unexpected use of shell wrapper during installation' >&2
    return 99
  fi
}
""")
        for command in ("cmake", "make", "ninja", "pkg-config", "xacro", "gazebo", "gzserver", "cc", "c++"):
            self.write_executable(self.prefix / "bin" / command, "#!/bin/bash\nexit 0\n")
        for command in ("python", "python3"):
            (self.prefix / "bin" / command).symlink_to("/usr/bin/python3")
        for module in (
            "numpy", "cv2", "cv_bridge", "message_filters", "tf2_ros", "yaml", "rclpy", "ament_index_python",
            "launch", "launch_ros", "control_msgs", "sensor_msgs", "trajectory_msgs", "moveit_msgs",
            "arm_description", "arm_control",
        ):
            module_path = self.prefix / "modules" / module
            module_path.mkdir(parents=True)
            (module_path / "__init__.py").touch()
        for module in ("arm_description/launch", "arm_description/resources", "arm_control/poses"):
            (self.prefix / "modules" / f"{module}.py").touch()
        (self.prefix / "modules/ament_index_python/packages.py").write_text("""import os
from pathlib import Path

def get_package_prefix(name):
    if name == os.environ.get('MOCK_MISSING_PACKAGE'):
        raise LookupError(name)
    return os.environ.get('MOCK_PACKAGE_PREFIX', os.environ['CONDA_PREFIX'])

def get_package_share_directory(name):
    if name == os.environ.get('MOCK_MISSING_WORKSPACE_PACKAGE'):
        raise LookupError(name)
    root = Path(os.environ['ARM_CONDA_PREFIX']).parent
    return str(root / 'install' / name / 'share' / name)
""")
        self.write_executable(self.prefix / "bin/ros2", """#!/usr/bin/env bash
set -eu
if [[ "$1" == pkg ]]; then
  [[ "$3" != "${MOCK_MISSING_PACKAGE:-}" ]] || exit 1
  echo "${MOCK_PACKAGE_PREFIX:-$CONDA_PREFIX}"
else
  [[ "$PATH:$PYTHONPATH:${AMENT_PREFIX_PATH:-}:$CMAKE_PREFIX_PATH:$LD_LIBRARY_PATH" != *'/opt/ros/'* ]]
  [[ "${AMENT_PREFIX_PATH:-}" != *'/install/old_package'* ]]
  [[ "$ROS_DOMAIN_ID" == 42 ]]
  printf 'launch %s\\n' "$*" >> "$MOCK_LOG"
fi
""")
        self.write_executable(self.prefix / "bin/colcon", """#!/usr/bin/env bash
set -eu
[[ "$PATH:$PYTHONPATH:${AMENT_PREFIX_PATH:-}:$CMAKE_PREFIX_PATH:$LD_LIBRARY_PATH" != *'/opt/ros/'* ]]
[[ "${AMENT_PREFIX_PATH:-}" != *'/install/old_package'* ]]
[[ "$LD_LIBRARY_PATH" == /unrelated/lib ]]
[[ "$ROS_DOMAIN_ID" == 42 ]]
printf 'build %s\\n' "$*" >> "$MOCK_LOG"
mkdir -p install
echo '# mock workspace overlay' > install/setup.bash
""")

    @staticmethod
    def write_executable(path, contents):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
        path.chmod(0o755)

    def run_script(self, name="install_dependencies.sh", *args, expected=0):
        result = subprocess.run(
            ["bash", str(self.root / "scripts" / name), *args],
            cwd=self.temp.name, env=self.env, text=True, capture_output=True,
        )
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        return result.stdout + result.stderr

    def test_create_and_install_all_groups(self):
        self.run_script()
        calls = self.log.read_text()
        self.assertIn("create --yes --prefix", calls)
        for dependency in ("ros-humble-ros-base", "ros-humble-moveit-ros-move-group", "gazebo=11.*"):
            self.assertIn(dependency, calls)
        self.assertIn(f"-DPython3_EXECUTABLE={self.prefix}/bin/python", calls)
        self.assertLess(calls.index("create --yes"), calls.index("build --symlink-install"))
        self.assertIn("--override-channels --strict-channel-priority", calls)
        self.run_script()
        self.assertIn("install --yes --prefix", self.log.read_text())

    def test_missing_ros_package_fails(self):
        self.env["MOCK_MISSING_PACKAGE"] = "controller_manager"
        output = self.run_script(expected=1)
        self.assertIn("[missing] controller_manager", output)

    def test_system_ros_package_cannot_pass_check(self):
        self.env["MOCK_PACKAGE_PREFIX"] = "/opt/ros/humble"
        self.assertIn("[missing] rclpy", self.run_script(expected=1))

    def test_python_import_failure_fails(self):
        (self.prefix / "modules/rclpy/__init__.py").write_text("raise ImportError('ABI mismatch')")
        self.assertIn("[missing] rclpy: ABI mismatch", self.run_script(expected=1))

    def test_system_executable_cannot_pass_check(self):
        (self.prefix / "bin/cmake").unlink()
        self.assertIn("[missing] cmake in", self.run_script(expected=1))

    def test_compilers_on_path_without_cc_cxx_variables(self):
        self.env["MOCK_NO_COMPILER_VARS"] = "1"
        output = self.run_script()
        self.assertNotIn("[missing]", output)

    def test_activation_hook_cannot_shadow_conda_tools(self):
        shadow = Path(self.temp.name) / "user-bin"
        self.write_executable(shadow / "colcon", "#!/bin/bash\nexit 99\n")
        self.env["MOCK_SHADOW_BIN"] = str(shadow)
        (self.prefix / "conda-meta").mkdir()
        self.run_script()
        self.assertIn("build --symlink-install", self.log.read_text())

    def test_compiler_fallback_cannot_use_system_compilers(self):
        self.env["MOCK_NO_COMPILER_VARS"] = "1"
        for command in ("cc", "c++"):
            (self.prefix / "bin" / command).unlink()
        output = self.run_script(expected=1)
        self.assertIn("[missing] CC compiler in", output)
        self.assertIn("[missing] CXX compiler in", output)

    def test_explicit_system_compiler_cannot_pass_check(self):
        self.env["MOCK_CC"] = "/usr/bin/cc"
        self.assertIn("[missing] CC compiler in", self.run_script(expected=1))

    def test_broken_compiler_cannot_pass_check(self):
        self.write_executable(self.prefix / "bin/cc", "#!/bin/bash\nexit 1\n")
        output = self.run_script(expected=1)
        self.assertIn("[missing] CC compiler is not runnable", output)
        self.assertIn("install_dependencies.sh", output)

    def test_install_failure_stops_before_activation(self):
        self.env["MOCK_INSTALL_FAIL"] = "1"
        self.run_script(expected=17)
        self.assertNotIn("activate", self.log.read_text())

    def test_refuses_conda_base(self):
        self.env["ARM_CONDA_PREFIX"] = str(self.base)
        self.assertIn("must not point to", self.run_script(expected=1))
        self.assertFalse(self.log.exists())

    def test_installer_rejects_all_arguments_without_installing(self):
        for argument in ("--unknown", "--check-only", "--with-moveit", "anything"):
            with self.subTest(argument=argument):
                self.run_script("install_dependencies.sh", argument, expected=1)
                self.assertFalse(self.log.exists())

    def test_help_needs_no_conda_and_preserves_workspace(self):
        artifact = self.root / "build/keep.txt"
        artifact.parent.mkdir()
        artifact.touch()
        self.env["CONDA_EXE"] = "/missing/conda"
        for script in ("install_dependencies.sh", "run_moveit.sh", "clean_ws.sh"):
            for flag in ("-h", "--help"):
                with self.subTest(script=script, flag=flag):
                    self.assertIn("Usage:", self.run_script(script, flag))
                    self.assertFalse(self.log.exists())
                    self.assertTrue(artifact.exists())

    def test_cleanup_rejects_unknown_arguments_before_removing_files(self):
        artifact = self.root / "build/keep.txt"
        artifact.parent.mkdir()
        artifact.touch()
        self.run_script("clean_ws.sh", "--unknown", expected=1)
        self.assertTrue(artifact.exists())

    def test_cleanup_preserves_conda_caches(self):
        env_cache = self.prefix / "lib/__pycache__/keep.pyc"
        project_cache = self.root / "src/__pycache__/remove.pyc"
        for path in (env_cache, project_cache):
            path.parent.mkdir(parents=True)
            path.touch()
        self.run_script("clean_ws.sh")
        self.assertTrue(env_cache.exists())
        self.assertFalse(project_cache.exists())

    def prepare_workspace(self):
        (self.prefix / "conda-meta").mkdir(exist_ok=True)
        setup = self.root / "install/setup.bash"
        setup.parent.mkdir(exist_ok=True)
        setup.write_text("# mock workspace overlay\n")
        for package, launch in (("arm_moveit_config", "moveit.launch.py"),
                                ("arm_moveit_config", "shelf_pick_place.launch.py")):
            path = self.root / "install" / package / "share" / package / "launch" / launch
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()

    def test_scripts_directory_has_only_three_entry_points(self):
        self.assertEqual({path.name for path in (REPO / "scripts").iterdir()}, {
            "clean_ws.sh", "install_dependencies.sh", "run_moveit.sh",
        })

    def test_launchers_only_launch_with_conda(self):
        self.prepare_workspace()
        for script, launch, option in (
            ("run_moveit.sh", "arm_moveit_config moveit.launch.py", "use_moveit_rviz:=true"),
        ):
            with self.subTest(script=script):
                self.run_script(script)
                calls = self.log.read_text()
                self.assertIn(f"launch launch {launch}", calls)
                self.assertIn(option, calls)
                self.assertNotIn("build ", calls)
                self.assertNotIn("--yes", calls)

    def test_launchers_report_missing_environment_without_installing(self):
        for script in ("run_moveit.sh",):
            with self.subTest(script=script):
                output = self.run_script(script, expected=1)
                self.assertIn("install_dependencies.sh", output)
                self.assertFalse(self.log.exists())

    def test_launcher_opens_rviz_with_background_physics(self):
        self.prepare_workspace()
        for options, launch in (
            ((), "moveit.launch.py"),
            (("--pick-place",), "shelf_pick_place.launch.py"),
        ):
            with self.subTest(options=options):
                self.run_script("run_moveit.sh", *options)
                calls = self.log.read_text()
                self.assertIn(f"arm_moveit_config {launch}", calls)
                if "--pick-place" in options:
                    self.assertIn("task_mode:=scene", calls)
                else:
                    self.assertNotIn("task_mode:=", calls)
                self.assertNotIn("use_gazebo:=", calls)
                self.assertNotIn("use_gazebo_gui:=", calls)
                self.assertIn("use_moveit_rviz:=true", calls)
                self.assertNotIn("build ", calls)
                self.assertNotIn("--yes", calls)
                self.log.unlink()

    def test_launcher_rejects_removed_and_unknown_flags_before_activation(self):
        self.env["CONDA_EXE"] = "/missing/conda"
        for flag in ("--headless", "--gazebo-gui", "--scene-only", "--plan-only", "--unknown"):
            with self.subTest(flag=flag):
                output = self.run_script("run_moveit.sh", "--pick-place", flag, expected=1)
                self.assertIn(f"Unknown option: {flag}", output)
                self.assertFalse(self.log.exists())

    def test_gazebo_requires_its_control_plugin(self):
        self.prepare_workspace()
        self.env["MOCK_MISSING_PACKAGE"] = "gazebo_ros2_control"
        output = self.run_script("run_moveit.sh", expected=1)
        self.assertIn("[missing] gazebo_ros2_control", output)
        self.assertNotIn("launch ", self.log.read_text())

    def test_launchers_report_missing_workspace_without_building(self):
        (self.prefix / "conda-meta").mkdir()
        for script in ("run_moveit.sh",):
            with self.subTest(script=script):
                output = self.run_script(script, expected=1)
                self.assertIn("install_dependencies.sh", output)
                calls = self.log.read_text()
                self.assertNotIn("build ", calls)
                self.assertNotIn("launch ", calls)
                self.assertNotIn("--yes", calls)

    def test_launcher_reports_missing_runtime_dependency_without_installing(self):
        self.prepare_workspace()
        self.env["MOCK_MISSING_PACKAGE"] = "controller_manager"
        output = self.run_script("run_moveit.sh", expected=1)
        self.assertIn("[missing] controller_manager", output)
        self.assertIn("install_dependencies.sh", output)
        self.assertNotIn("launch ", self.log.read_text())
        self.assertNotIn("--yes", self.log.read_text())

    def test_moveit_launcher_requires_moveit_dependencies(self):
        self.prepare_workspace()
        self.env["MOCK_MISSING_PACKAGE"] = "moveit_ros_move_group"
        output = self.run_script("run_moveit.sh", expected=1)
        self.assertIn("[missing] moveit_ros_move_group", output)
        self.assertNotIn("launch ", self.log.read_text())

    def test_launchers_do_not_require_build_tools_or_gazebo_gui(self):
        self.prepare_workspace()
        for command in ("colcon", "cmake", "make", "ninja", "cc", "c++", "gazebo"):
            (self.prefix / "bin" / command).unlink()
        for script in ("run_moveit.sh",):
            self.run_script(script)

    def test_launcher_reports_incomplete_build(self):
        self.prepare_workspace()
        launch = self.root / "install/arm_moveit_config/share/arm_moveit_config/launch/moveit.launch.py"
        launch.unlink()
        output = self.run_script("run_moveit.sh", expected=1)
        self.assertIn("[missing] Workspace package arm_moveit_config", output)
        self.assertIn("install_dependencies.sh", output)
        self.assertNotIn("launch ", self.log.read_text())

    def test_launcher_reports_broken_workspace_import(self):
        self.prepare_workspace()
        (self.prefix / "modules/arm_control/poses.py").write_text("raise ImportError('stale build')")
        output = self.run_script("run_moveit.sh", expected=1)
        self.assertIn("stale build", output)
        self.assertIn("install_dependencies.sh", output)
        self.assertNotIn("launch ", self.log.read_text())


if __name__ == "__main__":
    unittest.main()
