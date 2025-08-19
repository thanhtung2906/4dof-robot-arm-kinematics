import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
# === Ma trận biến đổi DH ===
def translation_matrix_zi(d):
    return np.array([[1,0,0,0],
                     [0,1,0,0],
                     [0,0,1,d],
                     [0,0,0,1]])

def rotation_matrix_zi(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                     [np.sin(theta),  np.cos(theta), 0, 0],
                     [0,              0,             1, 0],
                     [0,              0,             0, 1]])

def translation_matrix_xi(a):
    return np.array([[1,0,0,a],
                     [0,1,0,0],
                     [0,0,1,0],
                     [0,0,0,1]])

def rotation_matrix_xi(alpha):
    return np.array([[1, 0,              0,             0],
                     [0, np.cos(alpha), -np.sin(alpha), 0],
                     [0, np.sin(alpha),  np.cos(alpha), 0],
                     [0, 0,              0,             1]])


class Robotics():
    def __init__(self,dh_params):
        self.dh_params = dh_params
# === Động học thuận ===
    def forward_kinetic(self,custom_thetas = None):
        T = np.eye(4)
        joint_position = []
        for i,(theta,d,a,alpha) in enumerate(self.dh_params):
            if custom_thetas is not None:
                theta = custom_thetas[i]
            theta = np.deg2rad(theta)
            alpha = np.deg2rad(alpha)
            A = rotation_matrix_zi(theta) @ translation_matrix_zi(d) @ translation_matrix_xi(a) @ rotation_matrix_xi(alpha)
            T = T @ A
            joint_position.append(T[:3, 3])
        return T,joint_position
    
# === Động học nghịch 2 bậc tự do ===
    def inverse_kinetic_2DOF(self,px,py):
        _,_,l1,_ = self.dh_params[0]
        _,_,l2,_ = self.dh_params[1]

        c2 = (px**2 + py**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if abs(c2) > 1:
            raise ValueError("Điểm nằm ngoài workspace")

        s2 = np.sqrt(1 - c2**2)
        
        theta2_case_1 = np.arctan2(s2, c2)
        theta2_case_2 = np.arctan2(-s2, c2)
        
        k1 = l1 + l2 * c2
        k2 = l2 * s2

        k1_case2 = l1 + l2*c2
        k2_case2 = l2 * -s2
        
        theta1_case_1 = np.arctan2(py, px) - np.arctan2(k2, k1)
        theta1_case_2 = np.arctan2(py, px) - np.arctan2(k2_case2, k1_case2)

        return np.rad2deg(theta1_case_1),np.rad2deg(theta2_case_1), np.rad2deg(theta1_case_2),np.rad2deg(theta2_case_2)
    
# ===== Động học nghịch 3 bậc tự do ===== 

    def inverse_kinetic_3DOF(self,px,py,pz):
            _,d1,_,_ = self.dh_params[0]
            _,_,l2,_ = self.dh_params[1]
            _,_,l3,_ = self.dh_params[2] 

            theta1 = np.arctan2(py,px)

            pxy = np.sqrt(px**2+py**2)

            c3 = (pxy**2 + pz**2 - l3**2 - l2**2) / (2 * l2 * l3)
            if abs(c3) > 1:
                raise ValueError("Điểm nằm ngoài workspace")

            s3 = np.sqrt(1 - c3**2)
            
            theta3_case_1 = np.arctan2(s3, c3)
            theta3_case_2 = np.arctan2(-s3, c3)
            
            k1 = l2 + l3 * c3
            k2 = l3 * s3

            k1_case2 = l2 + l3*c3
            k2_case2 = l3 * -s3
            
            theta2_case_1 = np.arctan2(pxy, pz) - np.arctan2(k2, k1)
            theta2_case_2 = np.arctan2(pxy, pz) - np.arctan2(k2_case2, k1_case2)

            

            return np.rad2deg(theta2_case_1),np.rad2deg(theta3_case_1), np.rad2deg(theta2_case_2),np.rad2deg(theta3_case_2),np.rad2deg(theta1)
    

# ===== Động học nghịch 4 bậc tự do ======
 
    def inverse_kinetic_4DOF(self,px,py,pz):
            _,d1,_,_ = self.dh_params[0]
            _,_,a2,_ = self.dh_params[1]
            _,_,a3,_ = self.dh_params[2] 
            _,_,a4,_ = self.dh_params[3] 

            pz_adj = pz - d1
            
            # Tính Theta1 (Quay quanh trục Z)
            theta1 = np.arctan2(py,px)
            # Sử dụng Pytagore tính ra cạnh pxy và Pxy - l4 (trừ đi chiều dài của end-effector)
            pxy = np.sqrt(px**2+py**2) - a4
            # Kiểm tra điều kiện 
            distance = np.sqrt(pxy**2+pz_adj**2)
            max_reach = a2 + a3 
            min_reach = abs(a2-a3)
            c3 = (pxy**2 + pz_adj**2 - a2**2 - a3**2) / (2 * a2 * a3)
            c3 = np.clip(c3,-1,1)
            if abs(c3) > 1 or distance > max_reach or distance < min_reach:
                raise ValueError(f"Điểm nằm ngoài workspace. Distance ={distance}, Max reach ={max_reach}, Min reach = {min_reach}")
            # Tính s3 từ c3
            s3 = np.sqrt(1 - c3**2)
            # Từ s3 và c3 => theta3
            theta3_case_1 = np.arctan2(s3, c3)
            theta3_case_2 = np.arctan2(-s3, c3)
            # Đặt biến k1 k2 
            k1 = a2 + a3 * c3
            k2 = a3 * s3
            # Tính theta2 sử dụng tính góc argument
            gamma = np.arctan2(pz_adj, pxy)
            theta2_case_1 =  gamma - np.arctan2(k2, k1)
            theta2_case_2 =  gamma - np.arctan2(-k2, k1)
            # Tính theta 4 để end effector song song với mặt đất
            theta4_case1  = -(theta2_case_1 + theta3_case_1)
            theta4_case2  = -(theta2_case_2 + theta3_case_2)

            solution = []
            solution.append([np.rad2deg(theta1),np.rad2deg(theta2_case_1),np.rad2deg(theta3_case_1),np.rad2deg(theta4_case1)]) # Nghiệm 1
            solution.append([np.rad2deg(theta1),np.rad2deg(theta2_case_2),np.rad2deg(theta3_case_2),np.rad2deg(theta4_case2)]) # Nghiệm 2

            return solution
    def verify_answer(self, target, solution):
        position,_ = self.forward_kinetic(solution)
        calculated_pos = position[:3, 3]
        error = np.linalg.norm(calculated_pos - np.array(target))
        return calculated_pos, error
    def round_solution(self,solution):
        return [round(x) for x in solution]




def visualize_robot(joint_positions, title="Robot Visualization"):
    """
    joint_positions: list of numpy arrays (x, y, z) từ forward_kinetic
        """
        # Chèn gốc tọa độ (0,0,0) vào đầu danh sách
    positions = [np.array([0, 0, 0])] + joint_positions
    positions = np.array(positions)

        # Tạo figure 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

        # Vẽ các đoạn nối giữa các khớp
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                marker='o', color='blue', markersize=8, linewidth=2)

        # Đánh số khớp
    for i, (x, y, z) in enumerate(positions):
        ax.text(x, y, z, f"{i}", fontsize=10, color='red')

        # Đặt tiêu đề
    ax.set_title(title)

        # Giới hạn trục
    max_range = np.max(np.abs(positions)) * 1.2
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([0, max_range])

    # Nhãn trục
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()



