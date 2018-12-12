from setuptools import setup

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(name='cr_grasper',
      version='0.1',
      description='columbia robotics grasping simulator',
      long_description=long_description,
      long_description_content_type="text/markdown",
      url='https://github.com/carcamdou/cr_grasper',
      author='Carlyn Dougherty',
      author_email='ccd2134@columbia.edu',
      license='MIT',
      packages=['cr_grasper'],
      install_requires=[
            'numpy',
      ],
      zip_safe=False)


