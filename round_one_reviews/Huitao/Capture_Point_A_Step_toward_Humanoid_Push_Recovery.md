@def class = "journal"
@def authors = "Jerry Pratt; John Carff; Sergey Drakunov; Ambarish Goswami"
@def title = "Capture Point: A Step toward Humanoid Push Recovery"
@def venue = "IEEEXplore"
@def year = "2006"
@def hasmath = true
@def review = true
@def tags = ["Humanoid","Push recover","Bipedal walking"]
@def reviewers = ["Guan Huitao"]

\toc
### Broad area/overview
* This paper aim to solve the problem of “when and where to step” under a force disturbance using the concept of _Capture Points_. Instead of considering the arm as point mass, this paper replace it with a flywheel and thus, resulting a _"Linear Inverted Pendulum Plus Flywheel Model"_. Adding rotaional inertia enables the humanoid to control its centroidal angular momentum, much like the human beings do, significantly enlarging the Capture Region.
 ![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/4115555/4115556/4115602/4115602-fig-1-source-large.gif)
 _Abstract model of a flywheel body and a massless leg, the two actuators and CoM are located at flywheel center._
### Notation
* $x<sub>cpature</sub>$: Capture point
* $𝝉$: Joint torque
* $θ$: flywheel angle
* $X′,Ẋ′,t′,J′,τ′,θ′$: The dimensionless position, velocity, time, inertia, torque and angles.


### Specific Problem
* The conept of capture point is that _a humanoid must step to in order to come to a complete stop._ Using this concept, if the Capture point is within the convex hull of the foot support, the robot is able to recover without steps, otherwise it must take one step such that the base of support regains an intersenciton with the capture region. If the capture point is outside of the kinematic workspace of the swing foot, at least two steps are needed. Caption region leads to a principle approach to humanoid push recovery.
* When the flywheel is not available, the computed capture point x<sub>cpature</sub> is one single point. When the flywheel is made available, the Capture Point grow to a Capture Region, either a clockwise or counterclockwise acceleration will be required. Once the Capture Region is obtained, we could find whether the biped can recover without steps, one step or more than one steps.

### Solution Ideas
* A realistic troque-limited and anlge limited based Capture Region is computed. With a step torque due to the motor can chaieve max torque nearly instantly. We use τ<sub>min</sub> and τ<sub>max</sub> with the angle limit θ<sub>min</sub> and θ<sub>max</sub> the get the boundaty of the Capture Region.
* To prevent slipping, the ground reaction force vector must stay within the friction cone. A step change in torque could cause the ground reaction force to be horizontal, causing slipping on any non-attached surface.
* To reduce the variables involved, we use dimensionless analysis, the motion equation for Linear Inverted Pendulum Plus Flywheel Model become:

    $_**Ẍ′ = X′-τ′**_$

    $_**θ̈′ = τ′**_$

Thus, the only remaining parameters are τ′<sub>max</sub>, τ′<sub>min</sub>, θ′<sub>max</sub>, θ′<sub>min</sub>. Therefore, two Linear Inverted Pendulum Plus Flywheel system are dynamically same if they have the same values of those four variables.
![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/4115555/4115556/4115602/4115602-fig-4-source-large.gif)
![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/4115555/4115556/4115602/4115602-fig-5-source-large.gif)
_The figures show how the Capture Region changes due to the change of
τ′<sub>max</sub> and θ′<sub>max</sub>
![](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/4115555/4115556/4115602/4115602-fig-6-source-large.gif)
in the simulation, the robot is impulsively pushed at 0.5s, the robot recovers with a body lunge without a step.

### Comments
 * This paper present a method to find the Capture Region, and present a 2D biped balance recovery. The model use internal angular momentum to recover from a push after stepping into the Capture Region.
 * Actually in many cases it is not critical to stop in just one step, and the Capture Point is sensible to small errors. It could cause two or three steps when one step could have been possible.
 * We could have use max torque to determine the lunge time and rotation angle, we can also use maximum rotation angle to solve torque and lunge time.
