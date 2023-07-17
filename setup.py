from setuptools import setup

package_name = 'px4_offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yogesh',
    maintainer_email='170030012@iitdh.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = px4_offboard_control.offboard_control:main',
            'position_velocity_control = px4_offboard_control.position_velocity_controller:main',  
            'hover_test = px4_offboard_control.offboard_control_hover:main',
        ],
    },
)
