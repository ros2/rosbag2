from setuptools import setup

package_name = 'rosbag2_examples_py'

setup(
    name=package_name,
    version='0.22.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='geoff',
    maintainer_email='gbiggs@killbots.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_bag_recorder = rosbag2_examples_py.simple_bag_recorder:main',
            'data_generator_node = rosbag2_examples_py.data_generator_node:main',
            'data_generator_executable = rosbag2_examples_py.data_generator_executable:main',
        ],
    },
)
