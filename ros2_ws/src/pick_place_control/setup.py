from setuptools import find_packages, setup

package_name = 'pick_place_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/scene_objects.yaml']),
        ('share/' + package_name + '/launch', ['launch/pick_place_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saman-aboutorab',
    maintainer_email='saman.aboutorab@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planning_scene_loader = pick_place_control.planning_scene_loader:main',
            'pick_place_node = pick_place_control.pick_place_node:main',
            'pose_estimator = pick_place_control.pose_estimator:main',
            'yolo_detector = pick_place_control.yolo_detector:main',
            'vlm_detector = pick_place_control.vlm_detector:main',
        ],
    },

)
