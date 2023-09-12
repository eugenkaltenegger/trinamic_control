import os
from glob import glob
from setuptools import setup

package_name = 'trinamic_control'
  
setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ]
)