from setuptools import setup, find_packages
import os

package_name = 'task_6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        # Resource index file
        ('share/ament_index/resource_index/packages', 
         [os.path.join('resource', package_name)]),
        
        # Package.xml
        (os.path.join('share', package_name), 
         ['package.xml']),
        
        # Launch file (direct path specification)
        (os.path.join('share', package_name, 'launch'),
         [os.path.join('launch', 'red_ball_tracker.launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='izzy carlisle',
    maintainer_email='icarlisl@purdue.edu',
    description='Accomplishes task 6 of lab 4',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'red_ball_tracker = task_6.red_ball_tracker:main',
        ],
    },
)
