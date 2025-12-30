from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'depthai_cam'

# Build data_files dynamically to include all model files and subdirectories
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ('share/' + package_name + '/config', glob('config/*.rviz')),
]

# Recursively add all model files and directories
models_base = 'share/' + package_name + '/models'
for root, dirs, files in os.walk('models'):
    # Calculate the target path
    rel_path = os.path.relpath(root, '.')
    target_dir = models_base + '/' + rel_path.replace('models/', '')
    
    # Add all files in this directory
    if files:
        file_paths = [os.path.join(root, f) for f in files]
        data_files.append((target_dir, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group7',
    maintainer_email='shraypatel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'oak_publisher = depthai_cam.oak_publisher:main',
            'static_tf_publisher = depthai_cam.static_tf_publisher:main',
        ],
    },
)
