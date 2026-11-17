import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'guide_executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinyangshen',
    maintainer_email='19090888115@163.com',
    description='执行层：把 /guide_command 转成底盘速度指令',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'executor_node = guide_executor.executor_node:main',
            'mock_chassis_node = guide_executor.mock_chassis_node:main',
        ],
    },
)
