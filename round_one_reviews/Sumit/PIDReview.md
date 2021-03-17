@def review = true
@def tags = ["reviews","learning","control","analysis","pid"]
@def reviewers = ["Landon Clark","Sumit Kamat", "Joshua Ashley", "Daniel Kennedy"]
@def hasmath = true
@def class = "journal"
@def authors = "Cervantes, I.; Alvarez-Ramirez, J."
@def title = "On the PID tracking control of robot manipulators"
@def venue = "Elsevier"
@def year = "2000"

\toc


### Broad area/overview

This paper seeks to serve as a foundational piece of literature proving that, when tuned properly, a PID controller can guarantee semiglobal stability and small arbitrary errors in joint trajectories of robot manipulators. It also provides an effective, and mathematically proven, set of tuning guidelines for the PID parameters based on the system stability. Through the use of feedback compensation and a reduced-order error observer, the proposed solution was able to produce satisfactory results. 

### Notation

* This paper uses the Euler-Lagrangian model of dynamics for some basic suppositions, thus we have: $M(q)\ddot(q)$: inertia matrix, $C(q, \dot(q))\dot(q)$: Coriolis matrix, and $g(q)$: gravitational torques
* $\bar{M}$: estimate of the inertia matrix
* $\tau$: joint torque
* $q \in \mathbf{R}^{n}$: vector of joint positions
* $e_r$: error, difference between current joint value and expected joint value
* $n$: modeling error function (it's not really this n, but not sure what the real one is)
* $\bar{n}$: approximation on $n$, estimated via observer
* $A_c \in \mathbf{R^{2nx2n}}$: State matrix applied to the state of the transient performance of the system
* $x(t)$: transient system performance
* $\epsilon^{-1}$: adaption rate of the observer
  
### Specific Problem

This paper seeks to solve the problem of inconclusive results with PID controllers. Early on, it is stated that there lacks any concrete theoretical and practical results in current literature from this time proving the stability and effectiveness of the PID controller. This paper then defines its goals, sufficiently small tracking error, nonlocal stability guarantees, and "easy-to-use" tuning procedures. 

It should be noted that this review will gloss over the derivations in this paper, as this review would either turn directly into the paper itself or there would be too many jumps made along the way, the original meaning being lost via omission - either way invalidating deriving the multitude of equations derived in the paper.

The first proof of this paper seeks to show that their approach to the PID controller is equivalent to a classical PID controller. This is accomplished by deriving viable control gains based on the practical control law $\tau = \bar{M}[\ddot{q_r} - \bar{n} - Kx]$, $\bar{M}$, $\epsilon^{-1}$, and sub-matrices from $A_c$ ($K$ from the first equation is one of our submatrices in $A_c$). After this is proven, the paper then goes on to show that the system stability can be modeled as the system $\dot{x} = A_{c}x + He_0$ where $e_0$ is the estimation error $n - \bar{n}$. For the final step of proving the performance of their model, the paper seeks to prove semiglobal tracking. They first prove that the $A_c$ matrix has the properties of a global exponentially stable state matrix. They then prove that the origin of the tracking error is semiglobally practically stable by classical PID control. In other words, they show that given an area of interest, the PID gains computed earlier can produce a trajectory, that when compared to a spacial locality relative to the area of interest, this trajectory is roughly as similar to the area surrounding the computed area of interest as the actual area of interest compared to the area housing it (i.e. we have only some arbitrary errors in tracking). This is essentially semiglobal stability.

The final procedural portion of the paper deals with the tuning guidelines. There are three steps, simply put:
* Choose a $\bar{M}$ > 0
* Design the $K$ matrices from $A_c$ according to the link properties of the robot
* Choose a small enough $\epsilon$ value, as this will decrease the tracking error

$K_P = \bar{M}(K_1 + \epsilon^{-1}K_2)$
$K_I = \epsilon^{-1}\bar{M}K_1$
$K_D = \bar{M}(K_2 + \epsilon^{-1}I)$

The results shown in the end of the paper show promising results for a 2R robot manipulator. The tracking error is much better at a slow speed than a fast speed, but in either case it is sufficiently good, as well as stable. It can be seen that as $\epsilon$ decreases, so does the tracking error - although this does seem to fight against the ability for the controller to increase speed as quickly.

### Comments

* It would be interesting to perhaps see this approach applied to a higher DOF robot.
* Clearly this method has increased error at higher velocities, so is this method still viable for robots that operate in such environments?
* This paper does a very good job of moving from one mathematical formulation to another.

### Related Papers

Papers such as [this one](https://folk.ntnu.no/skoge/prost/proceedings/PID-12/papers/0096.pdf) have moved more towards the idea of autotuning PID parameters.
