from setuptools import setup

package_name = 'robomaster_tag'

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
    maintainer='robanatta',
    maintainer_email='robanatta@todo.todo',
    description='Static-map visual chaser for the RoboMaster CoppeliaSim scene.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'static_map_vision_chaser_node = robomaster_tag.static_map_vision_chaser_node:main',
        ],
    },
)
