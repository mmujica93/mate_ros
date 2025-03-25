from setuptools import setup, find_packages

setup(
    name='mate_ros',  # Replace with your package name
    version='0.0.0',
    packages=find_packages(include=['resources', 'resources.*']),
    install_requires=[],
    package_dir={'': '.'},  # The root is the base directory
)
