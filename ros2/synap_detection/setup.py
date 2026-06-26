from setuptools import setup

package_name = 'synap_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Teymur',
    maintainer_email='teymurmammadzada@yahoo.com',
    description='YOLO multi-target detector for the SynapLine tracker-lock workflow.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = synap_detection.yolo_detector_node:main',
        ],
    },
)
