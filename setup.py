from setuptools import find_packages, setup

package_name = 'gripper_helper'

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
    maintainer='final-project',
    maintainer_email='karamahati@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'contact_listener = gripper_helper.contact_listener:main',
            'wrapper_gripper_node = gripper_helper.wrapper_gripper_action:main',
            'gripper_service_server = gripper_helper.gripper_service_server:main',
            'contact_listener_wrapper = gripper_helper.contact_listener_wrapper:main',
        ],
    },
)
