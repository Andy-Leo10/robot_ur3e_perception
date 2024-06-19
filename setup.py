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
            'sim_dip        = robot_ur3e_perception.sim_dip:main',
            'sim_transform  = robot_ur3e_perception.sim_transform:main',
            'sim_marker     = robot_ur3e_perception.sim_marker:main',
            'sim_snapshot   = robot_ur3e_perception.sim_snapshot:main',

            'alt_yolov5     = robot_ur3e_perception.alt_yolov5:main',
            'alt_transform  = robot_ur3e_perception.alt_transform:main',
            'alt_marker     = robot_ur3e_perception.alt_marker:main',

            'real_yolov5     = robot_ur3e_perception.real_yolov5:main',
            'real_transform  = robot_ur3e_perception.real_transform:main',
            'real_marker     = robot_ur3e_perception.real_marker:main',
        ],
    },
)
