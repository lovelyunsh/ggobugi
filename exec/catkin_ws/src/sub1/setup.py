from setuptools import setup

package_name = 'sub1'

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
            'perception=sub1.perception:main',
            'publisher=sub1.publisher:main',
            'subscriber=sub1.subscriber:main',
            'controller=sub1.controller:main'
        ],
    },
)
