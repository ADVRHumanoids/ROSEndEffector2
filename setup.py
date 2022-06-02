import os
from glob import glob
from setuptools import setup

package_name = 'end_effector'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ]
)
