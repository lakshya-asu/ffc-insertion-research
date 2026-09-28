from setuptools import setup

setup(
    name="ffc_cell",
    version="0.1.0",
    packages=["ffc_cell"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ffc_cell"]),
        ("share/ffc_cell", ["package.xml"]),
        ("share/ffc_cell/launch", ["launch/perception.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "preprocess = ffc_cell.preprocess_node:main",
            "infer = ffc_cell.inference_node:main",
            "replay = ffc_cell.replay_node:main",
            "supervise = ffc_cell.supervisor_node:main",
        ]
    },
)
