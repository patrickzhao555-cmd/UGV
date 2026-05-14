from setuptools import find_packages, setup

package_name = "ugv_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    scripts=[
        "scripts/train_marker_model",
    ],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bluelule",
    maintainer_email="bluelule@todo.todo",
    description="Depth-based obstacle warning and simple ZED debug nodes",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "obstacle_warning = ugv_perception.obstacle_warning:main",
            "marker_vision_node = ugv_perception.marker_vision_node:main",
            "test_marker_vision = ugv_perception.marker_vision_test_node:main",
            "train_marker_model = ugv_perception.marker_model_trainer:main",
            "yolo_semantic_obstacles = ugv_perception.yolo_semantic_obstacle_node:main",
            "ugv_debug_dashboard = ugv_perception.ugv_debug_dashboard:main",
            "zed_obj_distance = ugv_perception.zed_obj_distance:main",
        ],
    },
)
