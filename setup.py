from setuptools import setup

package_name = 'ba_server_light'

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
    maintainer='Ferrypi',
    maintainer_email='Ferrypi@todo.todo',
    description='TODO: BA server light package to toggle docking and navigation light using relay board.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ferry_light = ba_server_light.ferry_light:main'
        ],
    },
)
