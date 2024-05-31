import os
from glob import glob
from setuptools import setup

package_name = 'seahawk'

setup(
    name=package_name,
    version='0.0.1',
    packages=['seahawk_deck', 'seahawk_rov', 'seahawk_deck.dash_styling', 'seahawk_deck.dash_widgets', 'seahawk_rov.i2c_sensors'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resources/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'resources'), glob(os.path.join('resources', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maximus',
    maintainer_email='matera@lifealgorithmic.com',
    description='TODO: Package description',
    license='AGPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "debug=seahawk_rov.debug:main",
            "claws=seahawk_rov.claws:main",
            "i2c=seahawk_rov.i2c:main",
            "thrust=seahawk_deck.thrust:main",
            "pilot_input=seahawk_deck.pilot_input:main",
            "rviz_markers=seahawk_deck.rviz_markers:main",
            'seahawk_rov = seahawk_rov:main',
            "dash=seahawk_deck.dash:main"
        ],
    },
)
