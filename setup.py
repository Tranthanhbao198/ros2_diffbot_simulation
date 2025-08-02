import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'diffbot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # ================================================================
        # BÂY GIỜ CHỈ CẦN CÀI ĐẶT MỘT THƯ MỤC LAUNCH DUY NHẤT
        # ================================================================
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        
        # Cài đặt các tệp dữ liệu khác
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tranthanhbao',
    maintainer_email='your_email@example.com',
    description='Simulation package for DiffBot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'performance_logger = diffbot_sim.performance_logger:main',
        ],
    },
)
