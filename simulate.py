import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time 

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
#p.loadSDF("box.sdf")
robotId = p.loadURDF("body.urdf")

duration = 2000

ps.Prepare_To_Simulate(robotId)

x = np.linspace(0,np.pi, duration)
y = np.sin(x)*np.pi/2
y2 = -np.sin(x)*np.pi/2
for l in range (1):
    for i in range(duration):
        ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                               jointName = b'Foot_Torso',
                               controlMode = p.POSITION_CONTROL,
                               targetPosition = y[i],
                               maxForce = 500)
        p.stepSimulation()
        time.sleep(1/500)
    for j in range(duration):
        ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                               jointName = b'FootTwo_Torso',
                               controlMode = p.POSITION_CONTROL,
                               targetPosition = y2[j],
                               maxForce = 500)
        p.stepSimulation()
        time.sleep(1/500)
    

p.disconnect()