from setuptools import setup

package_name = 'fedor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '/tcp_connector'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goodn',
    maintainer_email='goodn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fedor_connector = fedor_control.fedor_connector_node:main',
            'fedor_controller = fedor_control.fedor_controller_node:main',
        ],
    },
)
