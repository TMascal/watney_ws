from setuptools import setup

package_name = 'camera_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # Python package directory
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],  # Dependencies
    zip_safe=True,
    maintainer='Tim Mascal',
    maintainer_email='mascalt@my.erau.edu',
    description='A mixed C++ and Python ROS 2 package.',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_node = <package_name>.python_node:main',  # Python node entry point
        ],
    },
)