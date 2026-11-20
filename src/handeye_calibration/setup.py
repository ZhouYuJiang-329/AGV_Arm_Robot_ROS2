from setuptools import find_packages, setup

setup(
    name='handeye_calibration',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/handeye_calibration']),
        ('share/handeye_calibration', ['package.xml', 'README.md']),
        ('share/handeye_calibration/docs', ['docs/gui_preview.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zyj',
    maintainer_email='zhouyujiang73@gmail.com',
    description='Manual eye-in-hand calibration for arm2 and Astra S',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'handeye_node = handeye_calibration.node:main',
        'handeye_solve = handeye_calibration.solver:main',
        'handeye_gui = handeye_calibration.gui:main',
    ]},
)
