from setuptools import find_packages, setup

package_name = 'teensy_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bryson',
    maintainer_email='bryson.chen@student.unsw.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'util_arduino_node = teensy_pkg.util_arduino_node:main',
            'test_arduino_pub = teensy_pkg.test_arduino_pub:main',
        ],
    },
)
