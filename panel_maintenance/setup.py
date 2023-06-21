from setuptools import setup

package_name = 'panel_maintenance'

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
    maintainer='osboxes',
    maintainer_email='pedromfa98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'test_node = panel_maintenance.test_node:main',
             'routine = panel_maintenance.routine:main'
        ],
    },
)
