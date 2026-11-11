from setuptools import find_packages, setup

package_name = 'guide_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinyangshen',
    maintainer_email='19090888115@163.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dnn_stub_pub = guide_perception.dnn_stub_pub:main',
            'fake_image_pub = guide_perception.fake_image_pub:main',
            'guide_node = guide_perception.guide_node:main',
        ],
    },
)
