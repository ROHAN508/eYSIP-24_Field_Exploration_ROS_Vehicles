from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imu_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'description'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohan',
    maintainer_email='422PH5017@nitrkl.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['estimator = imu_pose_estimation.dist_vel_estimator:main',
                            'pub = imu_pose_estimation.pub:main',
                            'sub = imu_pose_estimation.sub:main'
        ],
    },
)
