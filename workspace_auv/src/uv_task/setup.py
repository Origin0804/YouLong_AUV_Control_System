from setuptools import setup

package_name = 'uv_task'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task_launch.py']),
        ('share/' + package_name + '/config', [
            'config/tasks.json',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Task execution package for YouLong AUV',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_runner = uv_task.task_runner:main',
        ],
    },
)
