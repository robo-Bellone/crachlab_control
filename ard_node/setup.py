from setuptools import setup

package_name = 'ard_node'  # Your package name
package_version = '0.1.0'  # Your package version

setup(
    name=package_name,
    version=package_version,
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',  # Add any other dependencies here
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Description of your package',
    license='License of your package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_publisher = ard_node.keyboard_publisher:main',
            'serial_publisher = ard_node.serial_publisher:main',
            'serial_node = ard_node.serial_node:main',  # Add this line 
        ],
    },
)

