## 模型

![机器人模型](.\figures\figure1.jpg)  
* 状态转移模型
$$
	x_t = g(v_t, x_{t-1},\Delta_t) + \epsilon_t
$$
式中$
x_t \coloneqq \left[ 
	\begin{array}{l}
		x_t \\
		y_t \\
		\theta_t
	\end{array}
	\right], \  g(v_t, x_{t-1},\Delta_t)  \coloneqq \left[ 
	\begin{array}{l}
		x_{t-1} + \cos \theta_{t-1} \cdot v_t  \Delta_t\\
		y_{t-1} + \sin \theta_{t-1} \cdot v_t \Delta_t\\\
		\theta_{t-1} + \omega_t \Delta_t
	\end{array}
	\right]
$，
$
\epsilon_t \coloneqq \begin{bmatrix}
		\epsilon_v \cdot \cos \theta_{t-1} \Delta_t \\
		\epsilon_v \cdot \sin \theta_{t-1} \Delta_t \\
		\epsilon_\omega \Delta_t
	\end{bmatrix}
$

* 观测模型：
$$
\mathbf{z}_t = h(\mathbf{x}_t) + \delta_t
$$
其中，$h(\mathbf{x}_t) \coloneqq \left[
	\begin{array}{c}
		x_t \\
		y_t
	\end{array}
	\right]$，$\delta_t\sim\mathcal{N}(0,Q_t)$:



## EKF(Extended Kalman Filter)主要流程，取$\Delta_t=0.5$推导
* 输入$\mu_{t-1},\Sigma_{t-1},u_t,z_t$
* $\bar{\mu}_t = g(u_t,\mu_{t-1})$
* $\bar{\Sigma}_t = G_t \Sigma_{t-1} G_t^T + R_t$
* $K_t = \bar{\Sigma}_t H_t^T (H_t\bar{\Sigma}_tH_t^T+Q_t)^{-1}$
	\item $\mu_t = \bar{\mu}_t + K_t (z_t -h(\bar{\mu}_t))$
* $\Sigma_t = (I-K_t H_t)\bar{\Sigma}_t$
* return $\mu_t, \Sigma_t$

其中，$G_t$为雅可比矩阵，$G_t \coloneqq \frac{\partial g}{\partial \mathbf{x}_{t-1}}$

$H_t$为观测模型的雅可比矩阵：
$
H_t \coloneqq \frac{\partial h}{\partial\textbf{x}_t}= \begin{bmatrix}
				\frac{\partial h_1}{\partial x_{t-1}} & \frac{\partial h_1}{\partial y_{t-1}} &  \frac{\partial h_1}{\partial \theta_{t-1}}\\
		\frac{\partial h_2}{\partial x_{t-1}} & \frac{\partial h_2}{\partial y_{t-1}} &  \frac{\partial h_2}{\partial \theta_{t-1}}\\
	\end{bmatrix} = \begin{bmatrix}
		1&0&0 \\
		0&1&0
	\end{bmatrix}
$


## EIF(Extended Information Filter)

引入信息矩阵$\Omega$，信息向量$\xi$，来估计高斯均值$\mu$和协方差矩阵$\Sigma$：
$$
	\Omega \coloneqq \Sigma^{-1}, \ \ \ \xi \coloneqq \Sigma^{-1} \mu
$$

同样需要雅可比矩阵：
$$
	G_t \coloneqq \frac{\partial g (u_t,x_{t-1})}{\partial x_{t-1}}, \ \ \ \ \ H_t\coloneqq \frac{\partial h(x_t)}{\partial x_t}
$$



EIF算法整体流程：
* 输入$\xi_{t-1},\Omega_{t-1},u_t,z_t$：
* $\mu_{t-1} = \Omega_{t-1}^{-1} \xi_{t-1}$
* $\bar{\Omega}_t = (G_t \Omega_{t-1}^{-1} G_t^T + R_t)^{-1}$
* $\bar{\xi}_t = \bar{\Omega}_t  g(u_t,\mu_{t-1})$
* $\bar{\mu}_t = g(u_t,\mu_{t-1})$
* $\Omega_t = H_t^T Q_t^{-1}H_t \bar{\Omega}_t$
* $\xi_t = \bar{\xi}_t + H_t^T Q_t^{-1} [z_t-h(\bar{\mu}_t)+H_t \bar{\mu}_t]$
* return $\xi_t, \Omega_t$


## 结果：
![EKF](.\figures\ekf_path.png)  

![EIF](.\figures\eif_path.png) 

![误差](.\figures\error.png)  