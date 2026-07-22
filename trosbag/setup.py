from setuptools import find_packages
from setuptools import setup

package_name = 'trosbag'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', [
            'launch/composable_recorder.launch.py',
            'launch/composable_player.launch.py',
        ]),
    ],
    install_requires=['ros2cli', 'ros2bag'],
    zip_safe=True,
    author='D-Robotics',
    maintainer='D-Robotics',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Entry point for trosbag CLI tools',
    long_description="""\
The package provides the tros command for recording and playing bag files.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'tros_bag = trosbag.command.tros_bag:TrosBagCommand',
        ],
        'ros2cli.extension_point': [
            'trosbag.verb = trosbag.verb:VerbExtension',
        ],
        'trosbag.verb': [
            'record = trosbag.verb.record:RecordVerb',
            'play = trosbag.verb.play:PlayVerb',
        ],
    }
)
