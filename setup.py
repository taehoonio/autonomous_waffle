from setuptools import find_packages, setup

package_name = 'nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ec327.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/ec327.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taehoon25',
    maintainer_email='taehoon25@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_node = nav_pkg.nav_node:main',
            'ctrl_node = nav_pkg.ctrl_node:main'
        ],
    },
)
