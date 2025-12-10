from setuptools import setup

package_name = 'rqt_fleet_view'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource', ['resource/FleetView.ui']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='min',
    maintainer_email='minjaejun291@gmail.com',
    description='RQT plugin showing a fleet leader/follower in 2D top view',
    entry_points={
        'console_scripts': [],
    },
)
