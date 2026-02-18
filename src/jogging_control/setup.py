from setuptools import find_packages, setup

package_name = 'jogging_control'

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
    maintainer='youssef',
    maintainer_email='Youssef.nagy073@gmail.com',
    description='Keyboard teleop for the ROAR 5-DOF arm',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'keyboard_jogger = jogging_control.keyboard_jogger:main',
        ],
    },
)