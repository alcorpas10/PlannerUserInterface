from setuptools import setup

package_name = 'task_point'

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
    maintainer='cvar',
    maintainer_email='60468917+alcorpas10@users.noreply.github.com',
    description='Project to check connection between user and drone swarm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = task_point.point_action_server:main',
        	'client = task_point.point_action_client:main',
        ],
    },
)
