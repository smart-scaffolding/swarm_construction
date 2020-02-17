Swarm Construction: Structure Module
=============

The structure module is used to control/simulate the home block for the purposes of Swarm Construction. It is what
keeps track of the robots, the structure being built, the blueprint of the structure to be built, etc.

Getting Started
---------------

These instructions will get you a copy of the project up and running on
your local machine for development and testing purposes. See installing
for notes on how to run the project on your system. See the `main documentation <https://smart-scaffolding.github
.io/swarm_construction/>`__ for more details on how to run the entire project.

Prerequisites
~~~~~~~~~~~~~

In order to run this project, it is best to set up a virtual environment so that all dependencies can be easily
installed. It is recommended to use an Anaconda environment, for which more instructions can be found at the
`Anaconda website <https://www.anaconda.com/>`__.

It should be noted that this is a large program that creates several threads and processes in order to run. Make sure
 that the hardware you are running it on is capable of multiple threads. It is also best to close out of unused
 applications and processes to ensure enough application memory.

Installing
~~~~~~~~~~

A step by step series of examples that tell you how to get a development
env running. First create a new conda environment with the specified python version:

::

    conda create --name <ENV NAME> python=3.7.6

Once the environment has been activated (conda active <ENV NAME>), run the following command in the directory where
you cloned the entire repository:

::

    pip install -r requirements.txt

After all of the dependencies have been installed, run the following command to get a robot up and running:

::
    python main.py

It should also be noted that config.py file specifies all of the configuration parameters for the module. These
values should be adjusted as needed. For example, if talking to other robots/the structure on other hardware, it is
important to change the IP address used to communicate between modules, found in the configuration.py file.


Built With
----------

-  Python (3.7.6)
-  VTK: The visualization toolkit, used to visualize the robot
-  ZMQ: ZeroMQ, the messaging protocol and medium used for communication/simulating communication


Authors
-------

-  **Caleb Wagner** - *Main Developer* -
   `Personal Website <calebtwagner.com`__


See also the list of
`contributors <https://github.com/smart-scaffolding/swarm_construction/contributors>`__ who
participated in this project.

License
-------

This project is licensed under the MIT License - see the
`LICENSE.md <LICENSE.md>`__ file for details

Acknowledgments
---------------

-  Swarm Construction 2020 MQP Team (Caleb Wagner, Hannan Liang, Neel Dhanaraj, Cameron Collins, Josue Contreras,
Trevor Rizzo)
-  Advisors: Carlo Pinciroli, Greg Lewin, Raghvendra Cowlagi

