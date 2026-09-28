from setuptools import setup

package_name = 'g1_multi_goal_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # 无自定义 Python 模块，留空
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'geometry_msgs', 'tf_transformations', 'paramiko', 'pyyaml'],
    zip_safe=True,
    maintainer='fk',
    maintainer_email='fk@todo.todo',
    description='Multi goal manager for G1 (ROS 2)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 声明节点入口（可选，与 install(PROGRAMS) 二选一，推荐后者）
            # 'g1_multi_goal_manager_node = scripts.g1_multi_goal_manager_node:main',
        ],
    },
)
