from setuptools import setup

setup(name='cr_grasper',
      version='0.1',
      description='columbia robotics grasping simulator',
      url='https://github.com/carcamdou/cr_grasper',
      author='Carlyn Dougherty',
      author_email='ccd2134@columbia.edu',
      license='MIT',
      packages=['cr_grasper'],
      install_requires=[
            'numpy',
      ],
      zip_safe=False)