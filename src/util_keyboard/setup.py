import os  
from glob import glob
from setuptools import find_packages, setup

package_name = 'util_keyboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={'': ['launch/*.py']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mitchell',
    maintainer_email='mitch.torok@gmail.com',
    description='Simple keyboard interface node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'util_keyboard = util_keyboard.util_keyboard:main'
        ],
    },
)
