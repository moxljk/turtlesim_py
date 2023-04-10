from setuptools import setup
from glob import glob

package_name = 'turtlesim_py'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('images', glob('images/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moxljk',
    maintainer_email='xxx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim = turtlesim_py.turtlesim:main'
        ],
    },
)
