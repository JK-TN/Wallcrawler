"""Wallcrawler_Ctrl controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
from controller import Supervisor
from controller import Keyboard
import numpy as np

supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())
print('Using timestep: %d' % timestep)

keyboard = Keyboard()
keyboard.enable(1) #sampling period (msec)


######## Setting up motors and position sensors ###############
m1 = supervisor.getDevice('m1') # Base motor (Controls Yaw)
m1.setPosition(float('inf'))
m1.setVelocity(0)

m1p = supervisor.getDevice('m1p') # Angle sensor for base motor
m1p.enable(timestep)

m2 = supervisor.getDevice('m2') # Middle joint motor
m2.setPosition(float('inf'))
m2.setVelocity(0)

m2p = supervisor.getDevice('m2p') # Middle joint angle sensor
m2p.enable(timestep)

m3 = supervisor.getDevice('m3') # Outermost joint motor
m3.setPosition(float('inf'))
m3.setVelocity(0)

m3p = supervisor.getDevice('m3p')  # Outermost motor angle sensor
m3p.enable(timestep)

########### IK Variables ##############
targetNode = supervisor.getFromDef('TARGET') # define yellow goal pose orb
Position = targetNode.getPosition() # Find initial goal pose in cartesian coordinates

#### DH Parameters ######
theta = [0,0,0]
a = [0.0254,0.05398,0.073025] # in meters
d = [0,0,0]
alpha = [math.pi/2,0,0] # in radians

gain = 10 #Gain for porpotional control (Modulate for oscillation compensation)

pose = [0,0,0] #Initial current pose

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    key = keyboard.getKey() # Get keyboard inputs
    
    #variables for keyboard manipulation
    targetNode = supervisor.getFromDef('TARGET') # define yellow goal pose orb
    trans_field = targetNode.getField('translation') # Translation orb field (Used to change orb position based on user input)
    rot_field = targetNode.getField('rotation') # Rotation orb field (Used to change orb orientation based on user input)
    trans_step = 0.01 # step sizes for translation and rotation inputs in meters
    rot_step = 0.01  
    
    #Q and W increments and sets x depending on user input
    if key == ord('Q'):
        Position = targetNode.getPosition()
        Position[0]-=trans_step
        trans_field.setSFVec3f(Position)
    elif key == ord('W'):
        Position = targetNode.getPosition()
        Position[0]+=trans_step
        trans_field.setSFVec3f(Position)
        
    #A and S increments and sets y depending on user input    
    if key == ord('A'):
        Position = targetNode.getPosition()
        Position[1]-=trans_step
        trans_field.setSFVec3f(Position)
    elif key == ord('S'):
        Position = targetNode.getPosition()
        Position[1]+=trans_step
        trans_field.setSFVec3f(Position)
    #Z and X increments and sets z depending on user input    
    if key == ord('Z'):
        Position = targetNode.getPosition()
        Position[2]-=trans_step
        trans_field.setSFVec3f(Position)
    elif key == ord('X'):
        Position = targetNode.getPosition()
        Position[2]+=trans_step
        trans_field.setSFVec3f(Position)
        
        
    ################# IK Calculations ###############################
    
    ############# Homogeneous Matricies For Joint-Joint Relations ######################
    H1_0 = np.matrix([[math.cos(theta[0]),-math.sin(theta[0])*math.cos(alpha[0]), math.sin(theta[0])*math.sin(alpha[0]),a[0]*math.cos(theta[0])],
                     [math.sin(theta[0]),math.cos(theta[0])*math.cos(alpha[0]),-math.cos(theta[0])*math.sin(alpha[0]),a[0]*math.sin(theta[0])],
                     [0,math.sin(alpha[0]), math.cos(alpha[0]),d[0]],[0,0,0,1]])

    H2_1 = np.matrix([[math.cos(theta[1]),-math.sin(theta[1])*math.cos(alpha[1]), math.sin(theta[1])*math.sin(alpha[1]),a[1]*math.cos(theta[1])],
                     [math.sin(theta[1]),math.cos(theta[1])*math.cos(alpha[1]),-math.cos(theta[1])*math.sin(alpha[1]),a[1]*math.sin(theta[1])],
                     [0,math.sin(alpha[1]), math.cos(alpha[1]),d[1]],[0,0,0,1]])

    H3_2 = np.matrix([[math.cos(theta[2]),-math.sin(theta[2])*math.cos(alpha[2]), math.sin(theta[2])*math.sin(alpha[2]),a[2]*math.cos(theta[2])],
                     [math.sin(theta[2]),math.cos(theta[2])*math.cos(alpha[2]),-math.cos(theta[2])*math.sin(alpha[2]),a[2]*math.sin(theta[2])],
                     [0,math.sin(alpha[2]), math.cos(alpha[2]),d[2]],[0,0,0,1]])
    
    ################ Transformation Matricies For Joint-World Relations #######################
    T2_0 = np.matmul(H1_0,H2_1)
    T3_0 = np.matmul(T2_0,H3_2)
    
    ######## Extracting Current Pose From Endeffector-World Matrix #########################
    pose[0] = T3_0.item(3)
    pose[1] = T3_0.item(7)
    pose[2] = T3_0.item(11)
    
    ####### Extracting Joint-World Rotation Matricies From Transformation Matricies ###########################
    R1_0 = np.matrix([[H1_0.item(0),H1_0.item(1),H1_0.item(2)],[H1_0.item(4),H1_0.item(5),H1_0.item(6)],[H1_0.item(8),H1_0.item(9),H1_0.item(10)]])
    R2_0 = np.matrix([[T2_0.item(0),T2_0.item(1),T2_0.item(2)],[T2_0.item(4),T2_0.item(5),T2_0.item(6)],[T2_0.item(8),T2_0.item(9),T2_0.item(10)]])
    
    
    ####### Extracting Joint-World Positions From Transformation Matricies #####################
    d3_0 = [T3_0.item(3),T3_0.item(7),T3_0.item(11)]
    d2_0 = [T2_0.item(3),T2_0.item(7),T2_0.item(11)]
    d1_0 = [H1_0.item(3),H1_0.item(7),H1_0.item(11)]
    
    
    ######## Developing Endeffector-Joint Deltas ##############
    d3_1 = np.asmatrix(np.subtract(d3_0,d1_0)).transpose()
    d3_2 = np.asmatrix(np.subtract(d3_0,d2_0)).transpose()
    
    ###### Developing Rotation Vectors For Joiunt Relations ###############
    rot1 = np.matrix([[0],[0],[1]])
    rot2 = np.matmul(R1_0,rot1)
    rot3 = np.matmul(R2_0,rot1)
    
    ###### Developing Jacobian columns ##############
    tran1 = np.cross(rot1.transpose(),np.matrix([d3_0])).transpose()
    tran2 = np.cross(rot2.transpose(),d3_1.transpose()).transpose()
    tran3 = np.cross(rot3.transpose(),d3_2.transpose()).transpose()
    
    ####### Constructing Jacobians ##############
    J = np.matrix([[tran1.item(0),tran2.item(0),tran3.item(0)],
                   [tran1.item(1),tran2.item(1),tran3.item(1)],
                   [tran1.item(2),tran2.item(2),tran3.item(2)]])
                   
    Position = targetNode.getPosition() # Get current goal pose
    goal = [Position[0],Position[1],Position[2]-0.166] # Format goal position vector accounting for platform height in meters
    
    diff_e = np.asmatrix(np.subtract(goal,pose)).transpose() # Find goal-current pose delta
    
    delta_e = gain*diff_e # multiply difference by gain
    
    delta_q  = np.matmul(np.matmul(np.linalg.pinv(np.matmul(J.transpose(),J)),J.transpose()),delta_e) # Derive joint velocities (rad/sec) using least squares
    
    if abs(diff_e.item(0)) >=0.001 or abs(diff_e.item(1)) >=0.001 or abs(diff_e.item(2)) >=0.001: # if current pose is not within specified tolerance of goal pose
        m1.setVelocity(0.0) # clear motor1 velocity values
        m1.setPosition(float('+inf')) # set position to undefined
        m1.setVelocity(delta_q.item(0)) #provide joint velocity
        
        m2.setVelocity(0.0) # clear motor2 velocity values
        m2.setPosition(float('+inf')) # set position to undefined
        m2.setVelocity(-delta_q.item(1))#provide joint velocity
        if m3p.getValue()<=0:
            m3.setVelocity(0.0) # clear motor3 velocity values
            m3.setPosition(float('+inf')) # set position to undefined
            m3.setVelocity(delta_q.item(2))#provide joint velocity
        else:
            m3.setVelocity(0.0) # clear motor3 velocity values
            m3.setPosition(float('+inf')) # set position to undefined
            m3.setVelocity(-delta_q.item(2))#provide joint velocity
        
        
    else: #otherwise set motor positions to a static position
        m1.setPosition(theta[0])
        m2.setPosition(-theta[1])
        m3.setPosition(theta[2])
    
  ##### Update Current Joint Angles #######################################
    theta[0] = m1p.getValue()
    theta[1] = -m2p.getValue()
    theta[2] = m3p.getValue()
    
# Enter here exit cleanup code.
