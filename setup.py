from setuptools import find_packages, setup

package_name = 'robotic_arm_servo_controller'

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
    maintainer='felipe',
    maintainer_email='lfeliperincons@gmail.com',
    description='Control del brazo mecanico RAS Junior con servos y ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodo que controla los 3 servos via Serial
            'joint_controller = robotic_arm_servo_controller.joints_keyboard:main',
        ],
    },
)
