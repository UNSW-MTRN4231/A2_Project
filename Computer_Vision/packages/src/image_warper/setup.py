from setuptools import find_packages, setup

package_name = 'image_warper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'opencv-python', 'std_msgs'],
    zip_safe=True,
    maintainer='mtrn',
    maintainer_email='z5362235@ad.unsw.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_warper = image_warper.image_warper:main',
            'ImagePublisher = image_warper.ImagePublisher:main',
        ],
    },
)
