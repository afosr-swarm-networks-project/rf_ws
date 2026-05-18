from setuptools import find_packages, setup

package_name = "rf_visualization"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}", ["plugin.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhongzheng Zhang",
    maintainer_email="zhangrenzhongzheng@outlook.com",
    description="Visualization nodes for RF detections",
    license="Apache-2.0",
    tests_require=["pytest"],
)
