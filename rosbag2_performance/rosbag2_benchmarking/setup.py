import os
from glob import glob
from setuptools import setup

package_name = 'rosbag2_benchmarking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), ['bundles/voyager/template.html'])
    ],
    install_requires=['setuptools', 'psutil>=5.4.2'],
    zip_safe=True,
    maintainer='pjaroszek',
    maintainer_email='piotrjaroszek@robotec.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_publishers = rosbag2_benchmarking.dummy_publishers:main',
            'raport_gen = rosbag2_benchmarking.raport_gen:main',
            'system_monitor = rosbag2_benchmarking.system_monitor:main',
            'voyager = rosbag2_benchmarking.voyager:main',
        ],
    },
)
