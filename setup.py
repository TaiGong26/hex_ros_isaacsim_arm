import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'hex_ros_isaacsim_arm'


def get_files(tar: str, src: str):
    all_paths = glob(f'{src}/*')

    data_files = []
    for path in all_paths:
        if os.path.basename(path) == '__pycache__':
            continue
        if os.path.isfile(path) and not path.endswith('.pyc'):
            data_files.append((tar, [path]))
        elif os.path.isdir(path):
            sub_files = get_files(f'{tar}/{os.path.basename(path)}', path)
            data_files.extend(sub_files)

    return data_files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *get_files('share/' + package_name, 'launch/ros2'),
        *get_files('share/' + package_name + '/config/ros2', 'config/ros2'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hexfellow',
    maintainer_email='thetaigon@qq.com',
    description='Isaac Sim bridge-forwarding package for the archer manipulator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'isaacsim_archer_y6 = hex_ros_isaacsim_arm.isaacsim_archer_y6:main',
        ],
    },
)
