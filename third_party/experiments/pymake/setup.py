"""cppmake auto-generates a big CMake file from C++ headers
"""
import setuptools

description = "Python Super CMake"

setuptools.setup(
    name='generate_cmake',
    version='1.6',
    license='MIT',
    long_description=__doc__,
    url='https://github.com/jpanikulam/experiments/tree/master/pymake',
    author='Jacob Panikulam',
    author_email='jpanikul@gmail.com',
    packages=setuptools.find_packages(),
    description=description,
    keywords="cmake make stupid",
    platforms='any',
    zip_safe=True,
    scripts=['scripts/pymake'],
    install_requires=[
        'colorama==0.3.9',
    ],

)
