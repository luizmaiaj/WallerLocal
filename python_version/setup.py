from setuptools import setup, find_packages

setup(
    name="robot_gp",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        'numpy>=1.21.0',
        'pillow>=8.3.0',
        'deap>=1.3.1',
    ],
)