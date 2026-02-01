import os
from setuptools import find_packages, setup

package_name = 'simulation_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include web directory recursively
        ('share/' + package_name + '/web', ['web/dist/index.html']),
        ('share/' + package_name + '/web/assets', [os.path.join('web/dist/assets', f) for f in os.listdir('web/dist/assets')]) if os.path.exists('web/dist/assets') else [],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Simulation Setup GUI',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = simulation_gui.gui_node:main'
        ],
    },
)
