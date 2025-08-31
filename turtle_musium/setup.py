from setuptools import find_packages, setup

package_name = 'turtle_musium'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='999aldwo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot9_main=turtle_musium.robot9_move:main',
            'tracker9=turtle_musium.tracker9:main',
            'detect9=turtle_musium.detect_publisher9:main',


        ],
    },
)
