from setuptools import setup
import os
from glob import glob

package_name = 'for_urdf_description'

def get_mesh_data_files():
    data_files = []
    for root, _, files in os.walk('meshes'):
        if not files:
            continue
        file_paths = [os.path.join(root, f) for f in files]
        install_dir = os.path.join('share', package_name, root)
        data_files.append((install_dir, file_paths))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        *get_mesh_data_files()
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
