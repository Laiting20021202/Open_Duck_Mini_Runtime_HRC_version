from pathlib import Path
from setuptools import setup

package_name = "duck_walk_bringup"

package_root = Path(__file__).parent
repo_root = package_root.parent

launch_files = [package_root / "launch" / "walk.launch.py"]
script_files = [repo_root / "scripts" / "walk_test.py"]

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [str(package_root / "resource" / package_name)]),
        (f"share/{package_name}", [str(package_root / "package.xml")]),
        (f"share/{package_name}/launch", [str(p) for p in launch_files]),
        (f"share/{package_name}/scripts", [str(p) for p in script_files]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Open Duck Maintainer",
    maintainer_email="maintainer@example.com",
    description="ROS 2 launch package that proxies to walk_test.py",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "walk = duck_walk_bringup.walk_entry:main",
        ],
    },
)
