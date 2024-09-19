from setuptools import find_packages, setup

package_name = 'tf_static_broadcast'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='e22750706642022@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "TF_StaticBroadcaster_Node = tf_static_broadcast.tf_static_publisher:main",
            "TF_Listener_Node = tf_static_broadcast.tf_static_listener:main"
        ],
    },
)
