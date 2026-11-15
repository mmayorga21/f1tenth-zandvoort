from glob import glob
from setuptools import setup

setup(name='f1tenth_controller', version='0.1.0', packages=['f1tenth_controller'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/f1tenth_controller']),
                  ('share/f1tenth_controller', ['package.xml', 'LICENSE']),
                  ('share/f1tenth_controller/config', glob('config/*.yaml')),
                  ('share/f1tenth_controller/launch', glob('launch/*.py'))],
      install_requires=['setuptools', 'numpy'], tests_require=['pytest'], zip_safe=True,
      maintainer='mmayorga21', maintainer_email='mmayorga21@users.noreply.github.com',
      description='Reactive Follow the Gap for Zandvoort', license='MIT',
      entry_points={'console_scripts': ['gap_follower = f1tenth_controller.gap_node:main']})
