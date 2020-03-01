#! /bin/sh -
cd /Users/calebwagner/SmartScaffoldingMQP_Code/zmq_vtk
export PYTHONPATH='/Users/calebwagner/SmartScaffoldingMQP_Code/zmq_vtk'
python components/simulator/main.py
python components/robot/main.py -i "ROBOT_1"