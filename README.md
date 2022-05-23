# rotms-slicer
A slicer extension for Robotic-TMS (Ro-TMS) paradigm.

# MedImgPlan
A slicer script module for medical image planning. 
## Configuration
- Please see MedImgPlan/Resources/Configs/Config.json for more details.
## Commands from Slicer
- Please see MedImgPlan/Resources/Configs/CommandsConfig.json for more details.
- This is the commands that will be sent from 3D slicer to the distant side.
- The unit of the numbers sent out in the commands are in ROS convention
- The commands should not exceed 150 chars (receiving side buffer)

# RobotControl
A slicer script module for robot controlling commands. 
