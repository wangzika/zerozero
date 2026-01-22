import numpy as np
import matplotlib.pyplot as plt

# 初始化参数
dt = 0.5
T = 100
N = int(T / dt)
acc = 0.5
t = np.arange(N) * dt

# 真值状态
A = np.array([[1, dt], [0, 1]])
B = np.array([[0.5 * dt ** 2], [dt]])
X_true = np.zeros((2, N))
for k in range(1, N):
    X_true[:, k] = A @ X_true[:, k - 1] + (B * acc).flatten()

# 模拟加速度计（含噪声和 bias）
bias_walk_std = np.sqrt(0.0009)
acc_noise_std = 0.5
acc_noise_std_discrete = acc_noise_std * np.sqrt( dt)
bias = 0
acc_measurements = np.zeros(N)
X_sim = np.zeros((2, N))

for k in range(1, N):
    bias += np.random.normal(0, bias_walk_std)
    noisy_acc = acc + bias + np.random.normal(0, acc_noise_std_discrete)
    acc_measurements[k] = noisy_acc
    X_sim[:, k] = A @ X_sim[:, k - 1] + (B * acc_measurements[k]).flatten()

    # X_sim[k]=


# 模拟速度观测值（含测量噪声）
vel_meas = X_true[1] + np.random.normal(0, 1.5, N)

# 卡尔曼滤波器函数
def run_kalman(Q_scale=1.0, R_scale=1.0):
    X_kf = np.zeros((2, N))
    P = np.eye(2) * 10
    Q = np.array([[0.01, 0], [0, 0.1]]) * Q_scale
    R = np.array([[1.5**2]]) * R_scale
    H = np.array([[0, 1]])

    for k in range(1, N):
        X_pred = A @ X_kf[:, k - 1] + (B * acc_measurements[k]).flatten()
        P_pred = A @ P @ A.T + Q

        y = vel_meas[k] - H @ X_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        X_kf[:, k] = X_pred + (K @ y).flatten()
        P = (np.eye(2) - K @ H) @ P_pred

    return X_kf

# 多组参数绘图
Q_scales = [0.001,0.1, 1,100,1000]
R_scales = [0.001, 1., 10,100,1000]
fig, axs = plt.subplots(4, 5, figsize=(18, 12))

# 第一行 + 第三行：改变 Q 影响
for i, q_scale in enumerate(Q_scales):
    X_kf = run_kalman(Q_scale=q_scale, R_scale=1.0)

    axs[0, i].plot(t, X_true[0], label='True Pos')
    axs[0, i].plot(t, X_kf[0], label='KF Pos')
    axs[0, i].plot(t, X_sim[0], label='sim Pos')
    axs[0, i].set_title(f'Q = {q_scale} ')
    axs[0, i].set_ylabel('Position (m)')
    axs[0, i].legend(fontsize=6)
    axs[0, i].grid()

    axs[2, i].plot(t, X_true[1], label='True Vel')
    axs[2, i].plot(t, X_kf[1], label='KF Vel',linestyle='-.')
    axs[2, i].plot(t, X_sim[1], label='sim Vel',linestyle='--')
    axs[2, i].plot(t, vel_meas, label='MEA Vel',linestyle=':',alpha=0.7,color="#2ED6D6")
    axs[2, i].set_title(f'Q = {q_scale} ')
    axs[2, i].set_ylabel('Velocity (m/s)')
    axs[2, i].legend(fontsize=6)
    axs[2, i].grid()

# 第二行 + 第四行：改变 R 影响
for i, r_scale in enumerate(R_scales):
    X_kf = run_kalman(Q_scale=1.0, R_scale=r_scale)

    axs[1, i].plot(t, X_true[0], label='True Pos')
    axs[1, i].plot(t, X_kf[0], label='KF Pos')
    axs[1, i].plot(t, X_sim[0], label='sim Pos')
    axs[1, i].set_title(f'R = {r_scale}')
    axs[1, i].set_ylabel('Position (m)')
    axs[1, i].legend(fontsize=6)
    axs[1, i].grid()

    axs[3, i].plot(t, X_true[1], label='True Vel')
    axs[3, i].plot(t, X_kf[1], label='KF Vel',linestyle='-.')
    axs[3, i].plot(t, X_sim[1], label='sim Vel',linestyle='--')
    axs[3, i].plot(t, vel_meas, label='MEA Vel',linestyle=':',alpha=0.7,color="#2ED6D6")
    axs[3, i].set_title(f'R  = {r_scale}')
    axs[3, i].set_ylabel('Velocity (m/s)')
    axs[3, i].set_xlabel('Time (s)')
    axs[3, i].legend(fontsize=6)
    axs[3, i].grid()

fig.suptitle('Effect of Process Noise Q(1) and Measurement Noise R(1) on Kalman Filter (Position and Velocity)', fontsize=16)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
