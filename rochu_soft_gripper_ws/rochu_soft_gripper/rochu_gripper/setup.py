from setuptools import setup

package_name = 'rochu_gripper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/example.launch.py']),
        ('share/' + package_name, ['launch/example.launch.xml']),
        ('share/' + package_name, ['config/params.yaml']),],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waihong',
    maintainer_email='waihongtan@live.com.my',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'rochu_gripper_node = rochu_gripper.rochu_gripper_fma5_node:main','rochu_logger_node = rochu_gripper.rochu_logger_debug:main'
        ],
    },
)