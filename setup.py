'''
Coloque este arquivo na raiz do seu pacote: ~/ros2_ws/src/py_srvcli/
'''
from setuptools import find_packages, setup

package_name = 'py_srvcli'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego de Miranda',
    maintainer_email='diego.demiranda@icloud.com',
    description='Exemplo de Cliente e Servidor em Python para ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = py_srvcli.calculator_server:main',
            'client = py_srvcli.calculator_client:main',
        ],
    },
)
