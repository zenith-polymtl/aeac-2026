from setuptools import find_packages, setup

package_name = 'shoot_and_capture'

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
    maintainer='joujou',
    maintainer_email='leducjulien169@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'shoot_controller = shoot_and_capture.shoot_controller:main',
            'image_capture = shoot_and_capture.image_capture:main',
            'image_transfer = shoot_and_capture.image_transfer:main',
        ],
    },
)
