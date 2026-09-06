import math
import time
import keyboard

### Robot IP - look for ours
ROBOT_HOST_IP = "192.168.1.254"

# # move with moveL
import rtde_control
rtde_c = rtde_control.RTDEControlInterface(ROBOT_HOST_IP)

#               x,y,z               rpy (roll (x), pitch (y), yaw (z)) -?
# The below code controlls the movement of the robot with cartesian coordinates.
# The following coordinates are a safe starting point for moveL ([0.25, -0.25, 0.35, 3.1416, 0, 0], 0.2, 0.1)
# The values does as follows:
# X moves planar to the Windows. The positive direction is away from the door
# Y moves towards the windows. The positive direction is towards the windows.
# Z moves up and down. The positive direction is upwards.
# Rotation X 
# Rotation Y 
# Rotation Z
# Speed
# Acceleration

#rtde_c.moveJ([0, -math.pi, math.pi/2, -math.pi/2, -math.pi/2, 0], 0.1, 0.02)

# The below code controlls the movement of the robot with joint angles.
# The following coordinates are a safe starting point for moveJ ([-0.4, -1.7, -1.2, -2, 1.5, -2], 0.2, 0.1)
# Each number is a value for the rotation of each joint (One rotation is 2*Pi)

#rtde_c.moveJ([0, -math.pi, math.pi/2, -math.pi/2, -math.pi/2, 0], 0.8, 0.4)



# # Get joint positions
import rtde_receive
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_HOST_IP)

# Move to the desired joint positions
def move_des(desired_joint_positions): 
    if desired_joint_positions:
        rtde_c.moveJ(desired_joint_positions, speed=0.7, acceleration=0.25)
    else:
        print("Inverse Kinematics solution not found!")
        
def wait_for_key(key="space"):
    print(f"Waiting for '{key}' key...")
    keyboard.wait(key)
    print(f"'{key}' key pressed!")

def djp(tcp_pose,current_joint_positions):
    desired_joint_positions = rtde_c.getInverseKinematics(tcp_pose, current_joint_positions)
    return desired_joint_positions    

# Calculate IK for the given TCP pose
# This function returns joint positions to achieve the desired TCP pose
djp_0 = djp([0.2  , 0, 0.4, -2.2552121731514405, 2.1838866669493817, -0.06527367547646405], rtde_r.getActualQ())
move_desired_0 = move_des(djp_0) # position over marble base
djp_1 = djp([0.3993951964494497, -0.2798580232847306, 0.3, -2.2552121731514405, 2.1838866669493817, -0.06527367547646405], rtde_r.getActualQ())
move_desired_1 = move_des(djp_1) # position over marble base
time.sleep(2)
djp_2 = djp([0.3993951964494497, -0.2791, 0.235, -2.2552121731514405, 2.1838866669493817, -0.06527367547646405], rtde_r.getActualQ())
move_desired_2 = move_des(djp_2) # lower to pick up marble base
time.sleep(5)
djp_2_1 = djp([0.3993951964494497, -0.2791, 0.27, -2.2552121731514405, 2.1838866669493817, -0.06527367547646405], rtde_r.getActualQ())
move_desired_2_1 = move_des(djp_2_1) # middle lift marble
djp_3 = djp([0.3993951964494497, -0.2798580232847306, 0.3, -2.2552121731514405, 2.1838866669493817, -0.06527367547646405], rtde_r.getActualQ())
move_desired_3 = move_des(djp_3) # lift the marble base
time.sleep(2)
djp_4 = djp([0.2576064929801978, 0.04640318015011849, 0.3, 2.2191525373307366, -2.218591622854392, 0.025637973761581438], rtde_r.getActualQ())
move_desired_4 = move_des(djp_4) # move the object to over the assembly fixture
time.sleep(2)
djp_5 = djp([0.2580064929801978, 0.04400318015011849, 0.23544694205390732, 2.2191525373307366, -2.218591622854392, 0.025637973761581438], rtde_r.getActualQ())
move_desired_5 = move_des(djp_5) # lower to release marble base
time.sleep(2)
djp_6 = djp([0.2566064929801978, 0.04640318015011849, 0.36, 2.2191525373307366, -2.218591622854392, 0.025637973761581438], rtde_r.getActualQ())
move_desired_6 = move_des(djp_6) # position over assembly
time.sleep(2)
djp_7 = djp([0.20042445314470567, -0.22230969950081841, 0.4, 2.227291531129319, 2.211939700926393, 0.004941294965979073], rtde_r.getActualQ())
move_desired_7 = move_des(djp_7) # position over silver cone 
time.sleep(2)
djp_8 = djp([0.20042445314470567, -0.22230969950081841, 0.3105732586021112, 2.227291531129319, 2.211939700926393, 0.004941294965979073], rtde_r.getActualQ())
move_desired_8 = move_des(djp_8) # lower to pick up silver cone
time.sleep(5)
djp_9 = djp([0.20042445314470567, -0.22230969950081841, 0.45, 2.227291531129319, 2.211939700926393, 0.004941294965979073], rtde_r.getActualQ())
move_desired_9 = move_des(djp_9) # lift the silver cone
time.sleep(2)
djp_10 = djp([0.25747879790076076, 0.040641858458700525, 0.38, -2.2242708629727175, 2.19801354240695, -0.003900237233785546], rtde_r.getActualQ())
move_desired_10 = move_des(djp_10) # position over assembly
time.sleep(2)
djp_11 = djp([0.25747879790076076, 0.040641858458700525, 0.3346326410665209, -2.2242708629727175, 2.19801354240695, -0.003900237233785546], rtde_r.getActualQ())
move_desired_11 = move_des(djp_11) # release the silver cone
time.sleep(5)
djp_12 = djp([0.25747879790076076, 0.040641858458700525, 0.38, -2.2242708629727175, 2.19801354240695, -0.003900237233785546], rtde_r.getActualQ())
move_desired_12 = move_des(djp_12) # position over assembly
time.sleep(2)
djp_13 = djp([0.19702478193746242, -0.32809679131604247, 0.32, 2.1965423453115416, 2.2212823180972756, -0.0025690155116824885], rtde_r.getActualQ())
move_desired_13 = move_des(djp_13) # over 7
djp_14 = djp([0.19702478193746242, -0.32809679131604247, 0.28007255634863637, 2.1965423453115416, 2.2212823180972756, -0.0025690155116824885], rtde_r.getActualQ())
move_desired_14 = move_des(djp_14) # pick up 7
time.sleep(5)
wait_for_key()
djp_14_extra = djp([0.19702478193746242, -0.32809679131604247, 0.33, 2.1965423453115416, 2.2212823180972756, -0.0025690155116824885], rtde_r.getActualQ())
move_desired_14_extra = move_des(djp_14_extra) # pick up 7 - midpoint
time.sleep(1)
djp_15 = djp([0.19702478193746242, -0.32809679131604247, 0.40, 2.1965423453115416, 2.2212823180972756, -0.0025690155116824885], rtde_r.getActualQ())
move_desired_15 = move_des(djp_15) # over 7
time.sleep(2)
djp_16 = djp([0.15729708565994466, 0.07529596822279454, 0.5397906324507137, 0.9836905137923578, 2.5516037867521386, 0.033546989516269714], rtde_r.getActualQ())
move_desired_16 = move_des(djp_16) # over the assembly 1
time.sleep(2)
djp_17 = djp([0.17505804656231835, 0.06696153048030926, 0.5291249914694922, 1.0140421723926838, 2.6234208935073546, 0.04973405063045551], rtde_r.getActualQ())
move_desired_17 = move_des(djp_17) # over the assenbly 2
time.sleep(1)
djp_18 = djp([0.22539528476526643, 0.04269787460505869, 0.48762654806036637, 1.0968896745073649, 2.81089995933692, 0.08695115009492947], rtde_r.getActualQ())
move_desired_18 = move_des(djp_18) # over the assembly 3
time.sleep(1)
djp_19 = djp([0.2434960615790283, 0.040848531102251735, 0.46984572941837527, 1.0735570524841662, 2.894058285295974, 0.08193520834912037], rtde_r.getActualQ())
move_desired_19 = move_des(djp_19) # over the assembly 4
time.sleep(1)
djp_20 = djp([0.25409476151004556, 0.04581500146278324, 0.402155168138056, 0.9950274948254809, 2.9704537619835407, -0.000403012645250943347], rtde_r.getActualQ())
move_desired_20 = move_des(djp_20) # over the assembly 5
time.sleep(5)
djp_21 = djp([0.18585601547611658, 0.07095327898161854, 0.5195823794255111, 0.9150524581737594, 2.7172290619047765, -0.009830444741404886], rtde_r.getActualQ())
move_desired_21 = move_des(djp_21) # over the assembly 6
time.sleep(2)
djp_22 = djp([0.48488530990958645, -0.018134769604416075, 0.3, -2.222768539769225, -2.17812360180389, 0.003162475426177084], rtde_r.getActualQ())
move_desired_22 = move_des(djp_22) # over the bottom cone
time.sleep(2)
djp_23 = djp([0.48488530990958645, -0.018134769604416075, 0.24858366515404673, -2.222768539769225, -2.17812360180389, 0.003162475426177084], rtde_r.getActualQ())
move_desired_23 = move_des(djp_23) # pick up the bottom cone
wait_for_key()
djp_24 = djp([0.48488530990958645, -0.018134769604416075, 0.3, -2.222768539769225, -2.17812360180389, 0.003162475426177084], rtde_r.getActualQ())
move_desired_24 = move_des(djp_24) # over the bottom cone
time.sleep(2)
djp_25 = djp([0.1972687614404333, -0.023440688964192956, 0.5586570412427576, 2.154632732654807, 1.2928285160644182, -0.031734001770562806], rtde_r.getActualQ())
move_desired_25 = move_des(djp_25) # midpoint path to assembly
time.sleep(2)
djp_26 = djp([0.18033987898246495, 0.044488578873292536, 0.524042139397563, 2.4370103336078404, 1.618674304477491, 0.4338039661314916], rtde_r.getActualQ())
move_desired_26 = move_des(djp_26) # over assembly 1
time.sleep(2)
djp_27 = djp([0.21203757843278956, 0.018991819640883666, 0.4974335540474018, 2.378326924692783, 1.9565972958639002, 0.3541565962231168], rtde_r.getActualQ())
move_desired_27 = move_des(djp_27) # over assembly 2
time.sleep(2)
djp_28 = djp([0.2456498306918776, 0.04847669456620357, 0.45065654329564436, 2.104896844802446, 2.2393604085317325, 0.07741838747406024], rtde_r.getActualQ())
move_desired_28 = move_des(djp_28) # over assembly 3
time.sleep(5)
djp_29 = djp([0.18033987898246495, 0.044488578873292536, 0.524042139397563, 2.4370103336078404, 1.618674304477491, 0.4338039661314916], rtde_r.getActualQ())
move_desired_29 = move_des(djp_29) # over assembly 4
time.sleep(2)   
