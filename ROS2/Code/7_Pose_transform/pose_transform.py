import transforms3d as tfs
import numpy as np

# 四元数转旋转矩阵
R1 = tfs.quaternions.quat2mat([1,0,0,0])
# 旋转矩阵转四元数
q1 = tfs.quaternions.mat2quat(R1)

# 四元数转轴角
k2 = tfs.quaternions.quat2axangle([1,0,0,0])
# 轴角转四元数
q2 = tfs.quaternions.axangle2quat([1,0,0],0.5)

# 固定轴欧拉角转四元数
q3 = tfs.euler.euler2quat(0,0,0,"sxyz")
# 四元数转固定轴欧拉角
r3 = tfs.euler.quat2euler([1,0,0,0],"sxyz")

# 固定轴欧拉角转旋转矩阵
R4 = tfs.euler.euler2mat(0,0,0,"sxyz")
# 旋转矩阵转固定轴欧拉角
r4 = tfs.euler.mat2euler(np.asarray([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]),"sxyz")

# 固定轴欧拉角轴角
k5 = tfs.euler.euler2axangle(0,0,0,"sxyz")
# 轴角转固定轴欧拉角
r5 = tfs.euler.axangle2euler([1,0,0],0.5,"sxyz")

# 轴角转旋转矩阵
R6 = tfs.axangles.axangle2mat([1,0,0],0.5)
# 旋转矩阵转轴角
k6 = tfs.axangles.mat2axangle(np.asarray([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]))