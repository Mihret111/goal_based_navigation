from setuptools import find_packages, setup

package_name = 'nav_ui'

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
    maintainer='mihret-k',
    maintainer_email='mihret.kochito16@gmail.com',
    description='Python UI node for the navigation project',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ui_node = nav_ui.ui_node:main',     # now, ui_node can be called by ROS2 from the terminal using 'ros2 run nav_ui ui_node'
        ],
    },
)
