#! /bin/sh -
cd /Users/calebwagner/SmartScaffoldingMQP_Code/zmq_vtk
export PYTHONPATH='/Users/calebwagner/SmartScaffoldingMQP_Code/zmq_vtk'
python components/simulator/robot_trajectory_serial_demo.py
python components/robot/robot_trajectory_serial_demo.py -i "ROBOT_1"