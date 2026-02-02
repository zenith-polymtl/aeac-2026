from setuptools import find_packages, setup

package_name = 'gimbal_controller'

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
    maintainer='guillaumegauthier',
    maintainer_email='guillaumegauthier3113@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gimbal_mavros = gimbal_controller.gimbal_mavros:main',
            'gimbal_test = gimbal_controller.gimbal_test_node:main'
        ],
    },
)
