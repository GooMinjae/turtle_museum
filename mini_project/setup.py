from setuptools import find_packages, setup

package_name = 'mini_project'

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
            'depth_to_3d=mini_project.3_1_b_depth_to_3d:main',
            'webcam = mini_project.webcam_detect_model:main',
            'main = mini_project.main_logic:main',
            'detect=mini_project.detect_publisher:main',
            'tracker=mini_project.tracker:main',

        ],
    },
)
