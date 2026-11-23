from glob import glob
from setuptools import setup, find_packages
setup(name='door_monitor', version='0.1.0', description='LaserScan-based monitoring of a known doorway with TF and stale-data handling',
      packages=find_packages(exclude=['test']), python_requires='>=3.10',
      install_requires=['PyYAML>=6'], extras_require={'test':['pytest']},
      license='Apache-2.0', maintainer='PyLoT Robotics', maintainer_email='163533636+PyLoT-Robotics@users.noreply.github.com',
      data_files=[('share/ament_index/resource_index/packages',['resource/door_monitor']),('share/door_monitor',['package.xml']),('share/door_monitor/config',glob('config/*')),('share/door_monitor/launch',glob('launch/*.py'))],
      entry_points={'console_scripts':['door_monitor = door_monitor.node:main', 'door-monitor-demo = door_monitor.demo:main']})
