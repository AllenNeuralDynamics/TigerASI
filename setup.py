import setuptools

with open("README.md", "r", newline="", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="TigerASI",
    version="0.0.5",
    author="Joshua Vasquez",
    author_email="joshua.vasquez@alleninstitute.org",
    description="a lightweight python interface for ASI brand Tiger Controllers",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/AllenNeuralDynamics/TigerASI",
    license="MIT",
    keywords= ['driver', 'asi', 'tigercontroller'],
    packages=setuptools.find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
    ],
    python_requires='>=3.9',
    install_requires=['pyserial', 'sphinx']
)
