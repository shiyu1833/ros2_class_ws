from setuptools import find_packages, setup
from glob import glob


package_name = 'ai_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebyy',
    maintainer_email='ebyy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'aicar_base = ai_car.aicar_base:main',
            'aicar_imu = ai_car.aicar_imu:main',
            'aicar_odom = ai_car.aicar_odom:main',
            'aicar_teleop = ai_car.aicar_teleop:main'
        ],
    },
)
