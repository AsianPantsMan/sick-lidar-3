from setuptools import find_packages, setup
from glob import glob # library for finding files
import os # library to interact with operating system like creating and removing files
package_name = 'retail_assistant_bringup'
#USES this file when you do colcon build to know how to build meaning what belongs to this package
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'description'),
            glob('description/*')),
        # Install RViz configs (optional)
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kendell',
    maintainer_email='kendelltaylor)0@gmail.com',
    description='Retail assistant robot package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
