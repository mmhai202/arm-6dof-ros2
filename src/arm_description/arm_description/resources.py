"""Read workspace assets from installed ROS packages or a source checkout."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import yaml


def package_file(package: str, relative_path: str) -> Path:
    try:
        directory = Path(get_package_share_directory(package))
    except PackageNotFoundError:
        directory = Path(__file__).resolve().parents[2] / package
    path = directory / relative_path
    if not path.is_file():
        raise FileNotFoundError(f"Missing {package} resource: {path}")
    return path


def load_yaml(package: str, relative_path: str) -> dict:
    with package_file(package, relative_path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}
