from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'amr_demo'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for folder in ['launch', 'config', 'maps', 'worlds']:
    files = [
        path for path in glob(os.path.join(folder, '*'))
        if os.path.isfile(path)
    ]
    if files:
        data_files.append((os.path.join('share', package_name, folder), files))

for folder in glob('textures/**/*', recursive=True):
    if os.path.isfile(folder):
        install_dir = os.path.join('share', package_name, os.path.dirname(folder))
        data_files.append((install_dir, [folder]))

for folder in glob('models/**/*', recursive=True):
    if os.path.isfile(folder):
        install_dir = os.path.join('share', package_name, os.path.dirname(folder))
        data_files.append((install_dir, [folder]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='test@todo.todo',
    description='Autonomous mobile service robot demo for EECE 698',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_landmark_detector = amr_demo.qr_landmark_detector:main',
            'go_to_pose = amr_demo.go_to_pose:main',
            'run_mission = amr_demo.run_mission:main',
            'mission_gui = amr_demo.mission_gui:main',
            'traffic_controller = amr_demo.traffic_controller:main',
            'traffic_random_nav_goals = amr_demo.traffic_random_nav_goals:main',
        ],
    },
)
