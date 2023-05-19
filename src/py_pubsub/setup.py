from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evgenii',
    maintainer_email='jekasjeny1012@gmail.com',
    description='Detection node',
    license='Apache License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = py_pubsub.DetectionNode:main',
            'image_invader = py_pubsub.PublisherNode:main',
        ],
    },
)
