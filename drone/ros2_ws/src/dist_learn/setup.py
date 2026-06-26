from setuptools import setup

package_name = "dist_learn"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="taisei",
    maintainer_email="taisei040428@gmail.com",
    description="SHIELD 用の動力学残差(disturbance)学習データ収集。"
                "実機ロールアウトから (x,u,d) を集めて CVAE 学習用 .npz を出す。",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # `ros2 run dist_learn collect_real_rollouts` で起動できる
            "collect_real_rollouts = dist_learn.collect_real_rollouts:main",
            "plot_rollouts = dist_learn.plot_rollouts:main",
            "to_shield_npz = dist_learn.to_shield_npz:main",
            "plot_shield_run = dist_learn.plot_shield_run:main",
        ],
    },
)
