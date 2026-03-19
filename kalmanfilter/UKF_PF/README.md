## 机器人模型

* 模型示意图：
![机器人模型](.\figures\figure1.jpg)  

* 观测模型和状态转移方程与EKF中一致

## UKF

引入 Unscented Transform 线性化系统方程，通过Sigma点来进行。如果系统状态向量长度为$n$，则共需要$2n+1$个Sigma点：

$$
\begin{array}{ll}
		\mathcal{X}^{[0]}=\mu & \\
		\mathcal{X}^{[0]}=\mu + \left[ \sqrt{(n+\lambda)\Sigma}\right]_i & \text{for}\ i = 1,\dots,n, \\
		\mathcal{X}^{[0]}=\mu - \left[ \sqrt{(n+\lambda)\Sigma}\right]_{i-n} & \text{for}\ i = n+1,\dots,2n, \\
\end{array}
$$

其中各式具体含义请参考相关书籍。

在进行线性化时：

$$
\mathcal{Y}^{[i]} = g(\mathcal{X}^{[i]}), \  i =0,\dots,2n
$$

从中得到均值和方差：
$$
\mu^\prime = \sum_{i=0}^{2n} w_{m}^{[i]} \mathcal{Y}^{[i]} \ , \ \Sigma^\prime = \sum_{i=0}^{2n} w_{c}^{[i]} (\mathcal{Y}^{[i]}-\mu^\prime) (\mathcal{Y}^{[i]} - \mu^\prime)^T.
$$
式中， $w_m^{[i]}, w_c^{[i]}, i=0,\dots,2n $为权重系数，具体取值为$w_m^{[0]}=\frac{\lambda}{n+\lambda},\ w_c^{[0]} =\frac{\lambda}{n+\lambda} + (1-\alpha^2+\beta), \ w_m^{[i]}=w_c^{[i]}=\frac{1}{2(n+\lambda)},\ i=1,\dots,2n.$

总结UKF($\mu_{t-1},\Sigma_{t-1},u_t,z_t$)算法具体流程如下：



*  $\mathcal{X}_{t-1} = (\mu_{t-1}\ \ \mu_{t-1} + \gamma \sqrt{{\Sigma_{t-1}}} \ \ \mu_{t-1}-\gamma \sqrt{\Sigma_{t-1}})$

* $\mathcal{\bar{X}}_{t}^* = g(u_t,\mathcal{X}_{t-1})$

* $\bar{\mu}_t = \sum_{i=0}^{2n}w_m^{[i]}\mathcal{\bar{X}}_{t}^{*[i]}$

* $\bar{\Sigma}_{t}=\sum_{i=0}^{2 n} w_{c}^{[i]}\left(\overline{\mathcal{X}}_{t}^{*[i]}-\bar{\mu}_{t}\right)\left(\overline{\mathcal{X}}_{t}^{*[i]}-\bar{\mu}_{t}\right)^{T}+R_{t}$

* $\bar{\mathcal{X}}_t = (\bar{\mu}_t \ \ \bar{\mu}_t + \gamma \sqrt{\bar{\Sigma}_t} \ \ \bar{\mu}_t - \gamma \sqrt{\bar{\Sigma}_t} )$

* $\overline{\mathcal{Z}}_{t}=h\left(\overline{\mathcal{X}}_{t}\right) $

* $\hat{z}_{t}=\sum_{i=0}^{2 n} w_{m}^{[i]} \overline{\mathcal{Z}}_{t}^{[i]}$

* $S_{t}=\sum_{i=0}^{2 n} w_{c}^{[i]}\left(\overline{\mathcal{Z}}_{t}^{[i]}-\hat{z}_{t}\right)\left(\overline{\mathcal{Z}}_{t}^{[i]}-\hat{z}_{t}\right)^{T}+Q_{t} $

* $\bar{\Sigma}_{t}^{x, z}=\sum_{i=0}^{2 n} w_{c}^{[i]}\left(\overline{\mathcal{X}}_{t}^{[i]}-\bar{\mu}_{t}\right)\left(\overline{\mathcal{Z}}_{t}^{[i]}-\hat{z}_{t}\right)^{T}$

* $K_{t}=\bar{\Sigma}_{t}^{x, z} S_{t}^{-1}$

* $\mu_{t}=\bar{\mu}_{t}+K_{t}\left(z_{t}-\hat{z}_{t}\right)$

* $\Sigma_{t}=\bar{\Sigma}_{t}-K_{t} S_{t} K_{t}^{T} $

* return $\mu_{t}, \Sigma_{t}$

----


## 粒子滤波(Particle Filter)

通过随机状态采样(粒子)，用于表示概率分布。粒子集合表示为：$\mathcal{X} \coloneqq \{x_t^{[1]} ,x_t^{[2]},\dots,x_t^{[M]}\}$，$M$为粒子个数。

粒子滤波需要计算通过粒子集置信度$bel(x_t)$，其具体算法流程为PF($\mathcal{X}_{t-1},u_t,z_t$)：

* $\bar{\mathcal{X}}=\mathcal{X}_t = \emptyset$
* for $m=1$ to $M$ do
* \ \ \ \ sample $x_t^{[m]}\sim p(x_t|u_t, x_{t-1}^{[m]})$
* \ \ \ \ $w_t^{[m]} = p(z_t | x_t^{[m]})$
* \ \ \ \ $\bar{\mathcal{X}}_t = \bar{\mathcal{X}}_t + \left< x_t^{[m]},w_t^{[m]}\right>$
* end for
* for $m=1$ to $M$ do
*  \ \ \ \ draw $i$ with probability $\propto \ w_t^{i}$
*  \ \ \ \ $\mathcal{X}_t =\mathcal{X}_t + x_t^{[i]} $
* end for
* return $\mathcal{X}_t$

## 仿真结果

![UKF](.\figures\ukf.png)

![PF](.\figures\pf.png)

