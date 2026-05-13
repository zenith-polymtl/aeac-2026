from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'upload_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'images'), glob('images/*.jpg')),
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
            'upload_controller = upload_controller.water_image_uploader:main',
            'send_photo_mock = upload_controller.send_photo_mock:main',
        ],
    },
)
