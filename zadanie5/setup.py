import os
from glob import glob
from setuptools import setup

package_name = 'zadanie5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madvx',
    maintainer_email='madvxpl@gmail.com',
    description='manipulator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ikin = zadanie5.ikin:main',
            'oint = zadanie5.oint:main'
        ],
    },
)
