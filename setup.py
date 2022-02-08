from setuptools import setup
from glob import glob

package_name = "volaly_kinematics"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch*")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf*")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gabri",
    maintainer_email="gabri@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    scripts=["scripts/pointing_node.py",
             "scripts/pointing_node_fast.py"],
)
