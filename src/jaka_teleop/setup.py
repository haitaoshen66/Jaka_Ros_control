from setuptools import setup
import glob

package_name = 'jaka_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],
    install_requires=['websockets', 'setuptools', 'opencv-python', 'PyYAML'],
    zip_safe=True,
    maintainer='DIJA',
    maintainer_email='1417599854@qq.com',
    description='Custom teleoperation nodes for JAKA control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_node = jaka_teleop.base_node:main',
            'button_node = jaka_teleop.button_node:main',
            'lift_node = jaka_teleop.lift_node:main',
            'gripper_node = jaka_teleop.gripper_node:main',
            'servo_control_node = jaka_teleop.servo_control_node:main',
            'workspace_zone_node = jaka_teleop.workspace_zone_node:main',
            'oculus_reader_node = jaka_teleop.oculus_reader_node:main',
            'camera_node = jaka_teleop.camera_node:main',
            'lerobot_recorder_node = jaka_teleop.lerobot_recorder_node:main',
            'lerobot_control_node = jaka_teleop.lerobot_control_node:main',
            # 'enable_node = jaka_teleop.enable_node:main',
        ],
    },
)
