from setuptools import find_packages, setup

package_name = "path_executor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="group7",
    maintainer_email="shraypatel@gmail.com",
    description="Path executor: follows nav_msgs/Path; hover on planning failure",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "path_executor_node = path_executor.path_executor_node:main",
        ],
    },
)
