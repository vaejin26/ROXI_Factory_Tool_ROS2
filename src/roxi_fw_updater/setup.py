from setuptools import setup

package_name = 'roxi_fw_updater'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='Firmware updater for ROXI devices using XMODEM',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fw_updater = roxi_fw_updater.fw_updater:main'
        ],
    },
)
