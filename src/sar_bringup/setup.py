import os
from glob import glob
from setuptools import setup

package_name = 'sar_bringup'

setup(
    name=package_name,
    version='0.0.0',

    packages=['sar_bringup'],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),

        (
            'share/' + package_name,
            ['package.xml'],
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='adwaith',
    maintainer_email='pkadwaith06@gmail.com',

    description='SAR bringup package',
    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
        ],
    },
)