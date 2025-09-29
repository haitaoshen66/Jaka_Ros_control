from setuptools import find_packages, setup

package_name = 'jaka_rviz_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'websockets',
        'setuptools'
    ],
    zip_safe=True,
    maintainer='pinnmax',
    maintainer_email='h765907397@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = jaka_rviz_client.client_member_function:main',
            'test_client = jaka_rviz_client.test:main',
            'oculus_reader_client = jaka_rviz_client.oculus_reader_client:main',
            'ros2_auto = jaka_rviz_client.ros2_auto:main',
            'jaka_servoj = jaka_rviz_client.jaka_servoj:main'
        ],
    },
)
