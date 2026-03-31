import numpy as np
from scipy.spatial.transform import Rotation as R

def poses_to_matrices(poses, degrees=True, order='xyz', inverse=False):
    mats = []
    for p in poses:
        # 两种格式兼容
        if (isinstance(p, (list, tuple, np.ndarray)) and len(p) == 2
                and hasattr(p[0], "__len__") and hasattr(p[1], "__len__")):
            pos = np.asarray(p[0], dtype=float).ravel()
            euler = np.asarray(p[1], dtype=float).ravel()
        else:
            arr = np.asarray(p, dtype=float).ravel()
            if arr.size != 6:
                raise ValueError("每个 pose 需要包含 6 个元素 (x,y,z,Rx,Ry,Rz)")
            pos = arr[:3]
            euler = arr[3:]

        Rm = R.from_euler(order, euler, degrees=degrees).as_matrix()
        # print("euler angles:", euler)
        # print("Rotation matrix:\n", Rm)
        # print()
        T = np.eye(4, dtype=float)
        T[:3, :3] = Rm
        T[:3, 3] = pos
        mats.append(T)

    if inverse:
        mats = [np.linalg.inv(T) for T in mats]
    return mats

tracker_poses = [
(np.array([-21.034, -756.726, -593.737]),np.array([43.348, 80.030, -2.216])),
(np.array([-10.862, -749.820, -562.040]),np.array([-130.519, 40.787, -174.765])),
(np.array([89.065, -768.606, -604.158]),np.array([-150.321, 45.747, 131.354])),
(np.array([77.250, -765.051, -590.323]),np.array([-9.576, 66.326, -75.130])),
(np.array([-28.351, -675.467, -684.257]),np.array([1.844, 36.922, -67.087])),
(np.array([41.095, -592.759, -654.313]),np.array([-65.625, 25.940, -136.389])),
(np.array([-90.364, -526.623, -736.256]),np.array([-110.498, -57.869, -136.122])),
(np.array([-113.155, -513.848, -609.751]),np.array([-65.840, 34.077, -126.764])),
(np.array([23.074, -613.539, -598.063]),np.array([-97.583, 63.571, -160.284])),
(np.array([-272.080, -747.891, -503.527]),np.array([-127.877, 69.450, -144.362])),
(np.array([239.108, -667.434, -672.001]),np.array([-79.751, 61.291, -174.437])),
(np.array([142.919, -742.259, -626.341]),np.array([-147.981, 64.137, -159.160])),
(np.array([230.652, -700.073, -570.929]),np.array([32.379, -35.439, 37.222])),
(np.array([201.557, -704.568, -720.080]),np.array([55.011, 0.668, 29.198])),
(np.array([50.811, -808.948, -677.868]),np.array([61.781, -4.852, 59.450])),

                ]
tcp_poses = [
(np.array([16.202, 27.961, 408.689]),np.array([140.041, 0.005, -30.028])),
(np.array([-13.159, 26.372, 414.198]),np.array([141.07, 1.013, 29.051])),
(np.array([-3.797, -80.622, 388.25]),np.array([167.741, -32.778, 6.119])),
(np.array([-18.197, -63.043, 393.666]),np.array([156.981, 21.517, -27.621])),
(np.array([102.861, 5.212, 489.119]),np.array([147.96, 46.518, -47.278])),
(np.array([53.571, -59.006, 566.827]),np.array([-177.314, 35.891, 35.034])),
(np.array([182.193, 36.579, 632.818]),np.array([-163.945, 20.436, 132.526])),
(np.array([59.779, 94.572, 654.039]),np.array([167.468, 38.118, 18.607])),
(np.array([7.765, -24.211, 546.832]),np.array([158.878, 5.369, 1.544])),
(np.array([7.9, 290.61, 437.168]),np.array([113.891, 3.989, -3.166])),
(np.array([6.525, -245.286, 471.974]),np.array([-170.2, 3.498, 2.179])),
(np.array([-8.077, -134.323, 404.936]),np.array([108.171, 0.663, 5.121])),
(np.array([-89.754, -201.556, 444.392]),np.array([144.114, -28.831, -150.232])),
(np.array([62.392, -222.039, 437.867]),np.array([144.785, -27.456, -109.971])),
(np.array([66.688, -59.877, 346.498]),np.array([162.946, -56.658, -121.518])),


                ]

# for p in tcp_poses:
#     p[1][0] = np.degrees(p[1][0])
#     p[1][1] = np.degrees(p[1][1])
#     p[1][2] = np.degrees(p[1][2])
#     p[0][0], p[0][1], p[0][2] = round(p[0][0]*1000, 3), round(p[0][1]*1000, 3), round(p[0][2]*1000, 3)
#     p[1][0], p[1][1], p[1][2] = round(p[1][0], 3), round(p[1][1], 3), round(p[1][2], 3)

print("TCP Poses:")
for p in tcp_poses:
    print(p)

tcp_poses = poses_to_matrices(tcp_poses, degrees=True, order='xyz')         # 机械臂是外部旋转，实际计算时用xyz
# tcp_poses = poses_to_matrices(tcp_poses, degrees=True, order='XYZ')         # 机械臂是外部旋转，实际计算时用xyz
tracker_poses = poses_to_matrices(tracker_poses, degrees=True, order='xyz') # vive也是外部旋转，实际计算时用xyz

for i in range(len(tcp_poses)):       # 计算每组姿态的 A_i
    B=tcp_poses[i][:3,:3]   # BA=C,A=C @ B^-1
    C=tracker_poses[i][:3,:3]
    A=C @ np.linalg.inv(B)          # A(base的原点在vive坐标系下的位置)
    # print("A:",A)
    # print("B:",B)
    # print("C:",C)
    # print()

M = np.zeros((3, 3))
for i in range(len(tcp_poses)):
    M += tcp_poses[i][:3,:3] @ tracker_poses[i][:3,:3].T  # 注意这里 B_i * C_i^T

U, S, Vt = np.linalg.svd(M)
A_est = Vt.T @ U.T

print("Estimated A(base的原点在vive坐标系下的位置):")
print(A_est)
euler_angles = R.from_matrix(A_est).as_euler('XYZ', degrees=True)
print(f"Rx={euler_angles[0]:.2f}°, Ry={euler_angles[1]:.2f}°, Rz={euler_angles[2]:.2f}°")

A_est_inv = np.linalg.inv(A_est)
print("Estimated A(vive的原点在base坐标系下的位置):")
print(A_est_inv)
euler_angles = R.from_matrix(A_est_inv).as_euler('XYZ', degrees=True)
print(f"Rx={euler_angles[0]:.2f}°, Ry={euler_angles[1]:.2f}°, Rz={euler_angles[2]:.2f}°")

for i in range(len(tcp_poses)):       # 计算每组姿态的 T
    T = tcp_poses[i][:3,3] - A_est_inv @ tracker_poses[i][:3,3]
    # print(f"T for pose {i+1}: {T}")

d_list = []
for i in range(len(tcp_poses)):
    d_i = tcp_poses[i][:3,3] - A_est_inv @ tracker_poses[i][:3,3]
    d_list.append(d_i)

T_est = np.mean(d_list, axis=0)

print("Estimated T:", T_est)

# print(R.from_euler('xyz', [90,-22.5,0], degrees=True).as_matrix())
# print(R.from_euler('xyz', [0,-22.5,0], degrees=True).as_matrix() @ R.from_euler('xyz', [90,0,0], degrees=True).as_matrix())
