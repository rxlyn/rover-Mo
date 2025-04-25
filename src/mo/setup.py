from setuptools import find_packages, setup

package_name = 'mo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan',
    maintainer_email='Ryan@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'controller = mo.controllerPublisher:main',
                'motor = mo.motorSubscriber:main',
            
                'commandLine = mo.commandLinePublisher:main',
                'ros2GUI = mo.ros2GUI:main',
                'keyboard = mo.keyboardControl:main', #Might be wrong name
                'sshConnect = mo.sshConnection:main',
                'GStream = mo.GStreamerImport:main',
                'stereoProcess = mo.stereoProcess:main',
                'autonomy = mo.autonomy:main',
        ],
    },
)
