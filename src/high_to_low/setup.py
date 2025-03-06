from setuptools import find_packages, setup

package_name = 'high_to_low'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial','tf_transformations'],
    zip_safe=True,
    maintainer='jake_161',
    maintainer_email='62108812+jake161@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'high_to_low_node = high_to_low.high_to_low_node:main'
        ],
    },
)