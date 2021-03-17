@def class = "journal"
@def authors = "Yun, X.; Bachmann E.R.;"
@def title = "Design, Implementation, and Experimental Results of a Quaternion-Based Kalman Filter forHuman Body Motion Tracking"
@def venue = "IEEE Transaction on Robotics"
@def year = "2006"
@def review = true
@def hasmath = true
@tags = ["reviews", "learning", "kalman filtering", "intertial sensors"]
@def reviewers = ["Josh Ashley", "Daniel Kennedy", "Landon Clark", "Huitao Guan"]

\toc

### Broad area/overview
This paper discusses the implementation of an extended Kalman filter (EKF) to create a complete Inertial Measurement Unit to model human limb motion. It is used as a composite of accelerometers, magnetometers, and MEMS sensors as inputs to the Kalman filter which then results in the output of a quaternion describing the orientation of the limb. 

### Notation
* $w$: Discrete white process noise
* $\omega$: Angular rate
* $q$: Quaternion in world frame
* $x$: State vector, $x$ = [$\omega_1$, $\omega_2$, $\omega_3$, $q_1$, $q_2$, $q_3$, $q_4$]
* $v$: White noise measurement
* $z_k$: observation of the true state $x_k$, $z_i$ = $x_i$ + $v_i$

### Specific Problem

The project aims to create an adequate implementation of IMU-based human limb motion tracking. The issues that arise with this come primarily from the intrinsic drift of the MEMS sensor to measure motion and the quadratic errors that come from double-integration on accelerometer measurements. 

The current Kalman filter sees this as a non-linear algebra problem due to the need for explicit conversion between the magnetic and gravitational fields into a quaternion vector described by the following equation. 

$x = \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ q_1 \\ q_2 \\ q_3 \\ q_4 \end{bmatrix}$

$\begin{bmatrix} z_1 \\ z_2 \\ z_3 \end{bmatrix} = \begin{bmatrix} x\ component\ of\ angular\ rate \\ y\ component\ of\ angular\ rate \\ z\ component\ of\ angular\ rate \end{bmatrix}$
$\begin{bmatrix} z_4 \\ z_5 \\ z_6 \end{bmatrix} = \begin{bmatrix} x\ component\ of\ acceleration \\ y\ component\ of\ acceleration \\ z\ component\ of\ acceleration\ \end{bmatrix}$
$\begin{bmatrix} z_7 \\ z_8 \\ z_9 \end{bmatrix} = \begin{bmatrix} x\ component\ of\ local\ magnetic field \\ y\ component\ of\ local\ magnetic\ field \\ z\ component\ of\ local\ magnetic\ field \end{bmatrix}$

An example of this problem comes from equation (7) in the paper. 

$z_7 = ((x^2_4 + x^2_7 - x^2_5 - x^2_6)/h_1 + 2(x_4*x_5 - x_6*x_7)/h_2 + 2(x_4*x_6 + x_5*x_7)/h_3)/(x^2_4 + x^2_5 + x^2_6 x^2_7) + v_4 

Where $h_1, h_2, h_3$ are the given values of the Earth’s magnetic field measured in the Earth’s coordinates and $v_4$ is the noise applied to $x_4$. 

While this gives a complete solution, the processing time of the non-linear equation must be account for every state transition which is highly costly. 

 ### Solution Ideas

Their solution attempts to linearize the conversion from the state space to observed space by solving for an estimated quaternion outside of the Kalman filter using the accelerometer and magnetic measurements. To do this they apply the QUEST algorithm which solves Wahba’s problem by calculating the four elements of the corresponding optimal quaternion. 

Now the conversion between the x and z space is completely linear by the following equation: 

$ z_i = x_i + v_i $ for $ i = 1, 2, 3, 4, 5, 6, 7 $ 

Additionally, the implementation of their Kalman filter is discussed. First they derive a discrete state transition matrix $\Phi_k$, an observation space map $H_k$ (which is the 7x7 identity matrix), the process noise covariance matrix $Q_k$, and the measurement noise covariance matrix $R_k$. These matrices are required for the initial steps of the EKF. Once the filter has been started, it will work to minimize the sum of squared errors, the performance of which can be measured by observing the error covariance matrix $P_k$. In the paper, the authors demonstrate that, when given an incorrect initial estimate, the filter is convergent upon the true value within a single iteration to a degree of acceptable accuracy (<1%).

In comparison to just using the QUEST algorithm and its corresponding quaternion, the Kalman filter approach was able to much more accurately provide a quaternion in periods of high motion and angular velocity. 


Additionally, the computation time of the filter algorithm is about 1.6ms on their implementation which is a 1MHz 16-bit RISC TI microcontroller. The total delay of the algorithm was roughly 100 ms, however most of this delay was attributed to transmission delay.

Noticeable lag between the measurements and simulation was not observed in experimentation leading them to denote the system as real-time. 

### Comments
* This paper utilizes the QUEST algorithm to develop an early estimate of quaternion, but there is no mention of how this works, nor a true description of computation cost apart from it “adding computational complexity”.
* The approach of feeding in a precomputed quaternion to linearize this problem is quite interesting as it linearizes an otherwise highly nonlinear problem.
* The results produced are dependent on specific operating conditions for the sensors involved, it is noted that any unideal conditions could render these algorithms ineffective.
* We are never provided with a runtime of the non-linear Kalman filter to provide a comparison for difference in performance of the two approaches. 

### Recent Papers
* This paper is an extension of the quaternion-based Kalman filtering method, yet again applied to human motion body tracking. [A New Quaternion-Based Kalman Filter for HumanBody Motion Tracking Using the Second Estimator of the Optimal Quaternion Algorithm and the JointAngle Constraint Method with Inertial andMagnetic Sensors](“https://res.mdpi.com/sensors/sensors-20-06018/article_deploy/sensors-20-06018-v2.pdf”)
