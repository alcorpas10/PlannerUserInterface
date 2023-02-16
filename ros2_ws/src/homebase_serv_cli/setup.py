from setuptools import setup

package_name = 'homebase_serv_cli'

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
    maintainer_email='60484182+GPatiA2@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'homebase_client = homebase_serv_cli.homebase_client:main',
        	'homebase_server = homebase_serv_cli.homebase_server:main'
        ],
    },
)