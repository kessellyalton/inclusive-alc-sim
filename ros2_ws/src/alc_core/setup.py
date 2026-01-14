import warnings
# Suppress harmless setuptools warnings about pytest-repeat
warnings.filterwarnings('ignore', category=UserWarning, module='setuptools.command.easy_install')

from setuptools import find_packages, setup

package_name = 'alc_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alton',
    maintainer_email='59618908+kessellyalton@users.noreply.github.com',
    description='Core nodes for adaptive learning and inclusive education simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'learner_model_node = alc_core.nodes.learner_model_node:main',
            'disability_sim_node = alc_core.nodes.disability_sim_node:main',
            'policy_node = alc_core.nodes.policy_node:main',
            'tutor_policy_node = alc_core.nodes.tutor_policy_node:main',
            'tutor_interface_node = alc_core.nodes.tutor_interface_node:main',
            'logger_node = alc_core.nodes.logger_node:main',
        ],
    },
)

