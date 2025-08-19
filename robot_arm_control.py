from dhtable import Robotics,visualize_robot
import time
import serial 
dh_params = [
    (0, 5.5,  0,90),   # theta1, d1, a1,      alpha1
    (0, 0,7.9, 0),   # theta2, d2, a2 = 10, alpha2
    (0, 0,  8.1, 0),   # theta3, d3, a3 = 7,  alpha3
    (0, 0,  4.5, 0)    # theta4, d4, a4 = 2,  alpha3
]
robot_arm = Robotics(dh_params=dh_params)
solution = robot_arm.inverse_kinetic_4DOF(px=3,py=8,pz=10)
T,joint_position = robot_arm.forward_kinetic(solution[1])

def round_solution(solution):
    return [round(x) for x in solution]

solution_0 = round_solution(solution=solution[1])
#visualize_robot(joint_positions=joint_position)
print(solution_0)

angles_int = [int(x) for x in solution[1]]
def theta2servo(angle_int):
    servo_shoulder = 180 -  angle_int[1] 
    servo_elbow =  180 - abs(angle_int[2])- (90 - angle_int[1] ) + 70
    servo_base = angle_int[0] + 90
    if servo_elbow > 180:
        raise ValueError("Angle can't be use")
    else:
        return servo_shoulder,servo_elbow,servo_base
servo_shoulder,servo_elbow,servo_base = theta2servo(angle_int=angles_int)
# ================ Kết nối sử dụng Serial ===========
# Mở kết nối
ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)

control_string = f"Z00A{servo_shoulder}B{servo_elbow}C{servo_base}\n"
print(f"Goc servo {servo_shoulder}, {servo_elbow} , {servo_base}")
# Điều khiển shoulder
ser.write(control_string.encode())
# Điều khiển elbow
ser.close()
visualize_robot(joint_position)
print("DONE")


