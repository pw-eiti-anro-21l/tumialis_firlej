from setuptools import setup

package_name = 'ZAD1'

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
    maintainer='tumialis123',
    maintainer_email='01149442@pw.edu.pl',
    description='Python parameter tutorial',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'zolwik = ZAD1.controls:main',
        ],
    },
)
