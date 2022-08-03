from setuptools import setup

package_name = 'mcity_proxy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan D. Lewis',
    maintainer_email='rdlrobot@umich.edu',
    description="ROS2 package providing all of the core functionality for Mcity's autonomous proxy.",
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcity_proxy = mcity_proxy.mcity_proxy:main'
        ],
    },
)
