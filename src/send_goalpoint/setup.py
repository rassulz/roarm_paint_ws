from setuptools import find_packages, setup

package_name = 'send_goalpoint'

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
    maintainer='rassul',
    maintainer_email='rassul@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_point = send_goalpoint.move_point:main',
            'grab_object_demo = send_goalpoint.grap_object_demo:main',
            'voltage_data = send_goalpoint.voltage_data:main',
        ],
    },
)
