from setuptools import setup

package_name = 'test_bridge'

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
    maintainer='kohei',
    maintainer_email='matsumoto@irvs.ait.kyushu-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_pub_node = test_bridge.test_pub_node:main',
            'test_sub_node = test_bridge.test_sub_node:main',
            'test_server_node = test_bridge.test_server_node:main',
            'test_client_node = test_bridge.test_client_node:main'
        ],
    },
)
