import numpy as np
import matplotlib.pyplot as plt

# 采样间隔
dt=0.5
T=100
N=int(T/dt)
# 设置真值
a_true=0.5
v_true=np.zeros(N)
v_sim=np.zeros(N)
p_true=np.zeros(N)
p_sim=np.zeros(N)
#加速度计噪声和bias
a_bias_std=np.sqrt(0.009)
a_niose=np.zeros(N)
a_bias=np.zeros(N)
#设置初值
a_bias[0]=0
v_true[0]=0
v_sim[0]=0
p_sim[0]=0
p_true[0]=0
a_py_noise=0.5
a_noise_std=a_py_noise*np.sqrt(1/dt)
a_mean=np.zeros(N)

#设置真值，sim值
for k in range(1,N):
    v_true[k]=v_true[k-1]+a_true*dt
    p_true[k]=p_true[k-1]+v_true[k-1]*dt
    # sim值
    a_bias[k]=a_bias[k-1]+np.random.normal(0,a_bias_std)
    a_mean[k]=a_true+a_bias[k]+np.random.normal(0,a_noise_std)
    v_sim[k]=v_sim[k-1]+a_mean[k]*dt
    p_sim[k]=p_sim[k-1]+v_sim[k-1]*dt

#======卡尔曼滤波======
#状态偏移矩阵
A=np.array([[1,dt],
            [0,1]])
B=np.array([[0.5*dt**2],
            [dt]])
x_est=np.zeros((2,N))
P_est=np.zeros((2,2,N))
# 噪声协方差矩阵，观测协方差矩阵
Q=([[0.001,0],
    [0.0001,0]])
R=([[1.5**2]])
# 观测矩阵
H=np.array([[0,1]])


x_est=np.zeros((2,N))
P_est=np.zeros((2,2,N))
x_est[:,0]=np.zeros(2)
P_est[:,:,0]=np.eye(2)
# 观测噪声
z=np.zeros(N)
z=v_true+np.random.normal(0,1.5,size=N)
for k in range(1,N):
    x_pre=A@x_est[:,k-1]+B.flatten()*a_mean[k]
    P_pre=A@P_est[:,:,k-1]+Q
    # 计算卡尔曼增益
    k_k=P_est[:,:,k-1]@H.T@np.linalg.inv(H@P_est[:,:,k-1]@H.T+R)
    x_est[:,k]=x_pre+k_k@(z[k]-H@x_pre)
    P_est[:,:,k]=(np.eye(2)-k_k@H)@P_pre

t=np.linspace(0,T,N)

plt.figure(figsize=(14, 6))
# 位置对比
plt.subplot(1,2,1)
plt.plot(t,p_true,label="true position")
plt.plot(t,p_sim,label="sim position",linestyle='--')
plt.plot(t,x_est[0],label="kalman filter position",linestyle='-.')

plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Position Estimation")
plt.legend()
plt.grid()
# 速度对比
plt.subplot(1,2,2)
plt.plot(t,v_true,label="true v")
plt.plot(t,v_sim,label="sim v",linestyle='--')
plt.plot(t,x_est[1],label="kf v",linestyle='-.')
plt.plot(t,z,label="mea v",linestyle=':')


plt.xlabel("Time (s)")
plt.ylabel("Velocity (m)")
plt.title("Velocity Estimation")
plt.legend()
plt.grid()
# 速度对比
plt.tight_layout()
plt.show()


    