from setuptools import find_packages, setup

package_name = 'tianyi2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/motor_config.yaml', 'config/joint_state_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/real_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='storm',
    maintainer_email='3347963340@qq.com',
    description='天轶2.0 MoveIt2 控制节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_goal_node = tianyi2_control.pose_goal_node:main',
            'bridge_node = tianyi2_control.bridge_node:main',
            'joint_state_publisher = tianyi2_control.joint_state_publisher:main',
        ],
    },
)
