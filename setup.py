from setuptools import setup

package_name = 'uav_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_detector.launch.py']),
        ('share/' + package_name + '/config', ['config/yolo_detector.yaml']),
        ('share/' + package_name, ['best.pt']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='level6',
    maintainer_email='your_email@example.com',
    description='Modular UAV perception nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector = uav_perception.yolo_detector:main',
        ],
    },
)
