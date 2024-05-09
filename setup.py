from setuptools import setup
from glob import glob
# to dissapear the warning of deprecated.
import warnings
warnings.filterwarnings("ignore")

package_name = 'robot_ur3e_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='andres.alamo@pucp.edu.pe',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_rgb = robot_ur3e_perception.camera_rgb:main',
            'camera_mono = robot_ur3e_perception.camera_mono:main',
            'camera = robot_ur3e_perception.camera:main',
            'transform = robot_ur3e_perception.transform:main',
            'marker = robot_ur3e_perception.marker:main',
        ],
    },
)
