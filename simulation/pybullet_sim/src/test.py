
import pybullet
import pybullet_data

import numpy as np
from numpy import pi as PI

import time

from pathlib import Path

from GlobalConstants import FootTrajectory
from GlobalConstants import Gait
from GlobalConstants import Mode

from Config import Config
from Robot import Robot
from IMU import IMU
from Command import Command
from MasterController import MasterController

if __name__ == "__main__":
    ### SIM SETUP ###
    # Connect to client in GUI mode
    physics_client = pybullet.connect(pybullet.GUI)
    pybullet.setPhysicsEngineParameter(enableFileCaching = 0)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_sim_id = pybullet.loadURDF("plane.urdf")
    ###
    
    # Robot
    robot = Robot(
        stance_polygon_length = 0.4,
        stance_polygon_width = 0.18,
        stance_height = 0.225
    )
    # Create configuration object which stores all configuration parameters used by almost everything
    config = Config()
    # Sensors
    imu = IMU(robot_sim_id = robot.sim_id, config = config)
    # Master Controller
    master_controller = MasterController(
        config = config,
        imu = imu,
        trajectory_shape = FootTrajectory.TRIANGULAR,
        use_capture_point = False,
        use_vpsp = False,
        use_tilt_stablisation = False
    )
    # Command
    command = Command()
    command.stance_polygon_length = 0.4
    command.stance_polygon_width = 0.18
    command.stance_height = 0.18
    command.mode = Mode.REST

    master_controller.stepOnce(robot, command)

    ### SIMULATE ###
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(0) # Step simulation only when setpSimulation() is called
    sim_duration = 3 # in seconds
    for i in range(int(5/config.dt)):
        pybullet.stepSimulation()
        time.sleep(config.dt)

    input() # Prevent closing simulator when script ends