import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aleks',
    maintainer_email='aleksandrpustrom@gmail.com',
    description='Robot description and static TF publisher for my_amr',
    license='Apache-2.0',
    entry_points={},
)
