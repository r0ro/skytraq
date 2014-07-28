from setuptools import setup

setup(name='skytraq',
      version='0.0.1',
      description='SkyTraq GPS binary protocal implementation',
      long_description='Implement SkyTraq Venus 6 GPS protocol',
      classifiers=[
        'Development Status :: 3 - Alpha',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.4',
        'Topic :: ',
      ],
      keywords='gps skytraq venus venus6 photo tagger',
      url='https://github.com/r0ro/skytraq',
      author='r0ro',
      author_email='r0ro@free.fr',
      license='MIT',
      packages=['skytraq'],
      install_requires=[
        'pyserial',
      ],
      include_package_data=True,
      zip_safe=False)
