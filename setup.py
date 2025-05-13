from setuptools import setup, find_packages

setup(
    name="odin",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "pyserial",
    ],
    extras_require={
        "dev": [
            "pytest",
            "jupyter",
            "matplotlib",
        ],
    },
    python_requires=">=3.6",
)