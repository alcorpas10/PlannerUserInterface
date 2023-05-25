from setuptools import setup

package_name = 'user_interface'

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
    maintainer='Alejandro Corpas Calvo',
    maintainer_email='al.corpas@alumnos.upm.es',
    description='User interface to integrate CVAR action clients',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_interface = user_interface.user_interface:main'
        ],
    },
)
