import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = 'fiducial_marker_pose'
SHARE_DIR = os.path.join('share', PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join(SHARE_DIR, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join(SHARE_DIR, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Evgenii Smirnov',
    author_email='jekasjeny1012@gmail.com',
    maintainer='Polytech Voltage Machine',
    maintainer_email='voltage.machine@yandex.ru',
    description='Detection and estimation of the pose of markers on the camera image',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_estimator_node = fiducial_marker_pose.marker_estimator_node:main',
        ],
    },
)
