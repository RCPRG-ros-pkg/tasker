from setuptools import setup

package_name = 'tasker'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@yourdomain.tld',
    description='The TaskER package',
    license='BSD',
    tests_require=['pytest'],
    scripts=['bin/task_harmonizer'],
)