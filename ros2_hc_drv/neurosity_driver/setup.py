from setuptools import setup

package_name = 'neurosity_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-dotenv', 'neurosity-sdk'],
    zip_safe=True,
    maintainer='Monica Perez-Serrano',
    maintainer_email='moperez@ethz.ch',
    description='ROS2 driver for Neurosity EEG',
    license='MIT',
    entry_points={
        'console_scripts': [
            'neurosity_driver = neurosity_driver.neurosity_driver:main',
        ],
    },
)
