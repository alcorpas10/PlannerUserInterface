from setuptools import setup

package_name = 'py_pubsub'

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
    maintainer='g.glenza',
    maintainer_email='g.glenza@alumnos.upm.es',
    description='pubsub python',
    license='BSD3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
