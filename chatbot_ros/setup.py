from setuptools import find_packages, setup

package_name = "chatbot_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="miguel",
    maintainer_email="mgons@unileon.es",
    description="TODO: Package description",
    license="GPL-3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chat_bot_node = chatbot_ros.chat_bot_node:main",
        ],
    },
)
