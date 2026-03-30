from setuptools import setup

package_name = 'go2_teleop_ctrl_keyboard'

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
    maintainer='ciao',
    maintainer_email='ciao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_teleop_ctrl_keyboard = go2_teleop_ctrl_keyboard.go2_teleop_ctrl_keyboard:main'
        ],
    },
)
