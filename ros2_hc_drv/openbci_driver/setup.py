from setuptools import setup

package_name = 'openbci_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'numpy', 'pyserial'],
    zip_safe=True,
    maintainer='Monica Perez-Serrano',
    maintainer_email='moperez@ethz.ch',
    description='ROS 2 driver for OpenBCI Cyton boards using healthcare_msgs/EEG',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openbci_driver = openbci_driver.openbci_driver:main',
        ],
    },
)
