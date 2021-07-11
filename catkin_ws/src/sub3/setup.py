from setuptools import setup

package_name = 'sub3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='mgko@morai.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_detector = sub3.tf_detector:main',
            'client = sub3.client:main',
            'connect_model = sub3.connect_model:main',
            'iot_udp = sub3.iot_udp:main',
            'run_localization= sub3.run_localization:main',
            'run_mapping= sub3.run_mapping:main',
            'random_move = sub3.random_move:main'

        ],
    },
)
