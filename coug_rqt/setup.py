from setuptools import find_packages, setup
import os
from glob import glob

package_name = "coug_rqt"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "plugin.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "ui"), glob("ui/*.ui")),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml")
            + glob("config/*.yaml.template")
            + glob("config/*.perspective"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="snelsondurrant",
    maintainer_email="snelsond@byu.edu",
    description="CoUGARs RQT dashboards and UI monitoring tools.",
    license="Apache-2.0",
    url="https://github.com/cougars-auv/coug_rqt",
    extras_require={
        "test": [
            "pytest",
            "pytest-cov",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
