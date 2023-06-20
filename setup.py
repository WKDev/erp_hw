import os
from glob import glob
from setuptools import setup

package_name = 'hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # launch script 230620chson
        ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='chanhyukman@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_driver = hw.imu_driver:main',
            'erp_driver = hw.erp_driver:main',
        ],
    },
)
