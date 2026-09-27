from setuptools import find_packages, setup

package_name = 'day14_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shahzaib',
    maintainer_email='talha.aamer.23@gmail.com',
    description='Day 14 programmatic navigation using Nav2',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'nav_client = day14_nav.nav_client:main',
        'fleet_manager = day14_nav.fleet_manager:main',
    ],
},
)