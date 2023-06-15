import os
from setuptools import setup
from setuptools import find_packages

package_name = '04_risa_racing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name+'/utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Konrad',
    maintainer_email='konrad1borowik@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = 04_risa_racing.main:main',
            'planner = 04_risa_racing.planner:main'
        ],
    },
)
