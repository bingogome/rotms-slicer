# rotms-slicer
A slicer extension for Robotic-TMS (Ro-TMS) paradigm.
![alt text](https://github.com/bingogome/documents/blob/main/rotms-slicer/rotms-ui.png)
![alt text](https://github.com/bingogome/documents/blob/main/rotms-slicer/rotms-visual.png)
# MedImgPlan
A slicer script module for medical image planning. 

## Configuration
- Please see MedImgPlan/Resources/Configs/Config.json for more details.

## Commands from Slicer
- Note:
  - Please see MedImgPlan/Resources/Configs/CommandsConfig.json for more details.
  - This is the commands that will be sent from 3D slicer to the distant side.
  - The unit of the numbers sent out in the commands are in ROS convention
  - The commands should not exceed 256 chars (receiving side buffer)
- Commands encoding:
  - "START_AUTO_DIGITIZE": "start_autodigitzat",
  - "START_REGISTRATION": "start_registration",
  - "USE_PREV_REGISTRATION": "start_useprevregis",
  - "CURRENT_LANDMARK_ON_IMG": "img_fid_pnt",
  - "NUM_OF_LANDMARK_ON_IMG": "img_fid_num",
  - "TARGET_POSE_ORIENTATION": "target_rot",
  - "TARGET_POSE_TRANSLATION": "target_tsl"
}

# RobotControl
A slicer script module for robot controlling commands. 

## Configuration
- Please see RobotControl/Resources/Configs/Config.json for more details.

## Commands from Slicer
- Note:
  - Please see RobotControl/Resources/Configs/CommandsConfig.json for more details.
  - This is the commands that will be sent from 3D slicer to the distant side.
  - The unit of the numbers sent out in the commands are in ROS convention
  - The commands should not exceed 256 chars (receiving side buffer)

# TargetVisualization
