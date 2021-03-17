@def review = true
@def tags = ["reviews","learning","control","analysis","actuation"]
@def reviewers = ["Landon Clark"]
@def hasmath = true
@def class = "journal"
@def authors = "Pratt, G. A.; Williamson, M. M."
@def title = "Series Elastic Actuators"
@def venue = "IEEE"
@def year = "1995"

\toc


### Broad area/overview

This paper looks to consider practical applications of series elastic actuators as well as control methods for such. By using elastic actuators, the control problem of each joint is transformed from position to force control. This happens as we can model positional differences between our load and motor values as the displacement across our spring, thus providing a very simple calculation to find the force applied at the link. While it is simple to model what the force will be at the load, it is far less so to properly control the force applied at the load. Thus the paper details a control method to provide stable, accurate control of the forces applied at the load.

### Notation

* $K_s$: elasticity spring rate (otherwise referred to as a spring constant)
* $f_l, F_l$: force applied to the load
* $f_m, F_m$: force applied to the motor
* $x_l, X_l$: position of the load
* $x_m, X_m$: position of the motor
* $M_m$: motor mass
* $Z$: mechanical impedance
  
### Specific Problem

To properly control the force output on the load, the paper seeks to relate $Z$ to $F_l$. By doing this, they were able to find performance characteristics of the actuator relative to the interface resonance. Once the model for forces was developed, a control method was necessary to fulfill their goals. 

### Solution Ideas

With respect to the analysis of the motor force and impedance, it should be noted that when compared to an inelastic actuator, an elastic actuator is able to, when below interface resonance, provide maximal motor force for virtually every possible motor impedance except for those close to $Z = 0$. However, the zero of the stiff actuator moves from $Z = 0$ to $Z = M_m\omega^{2}$, a fact that I believe is referencing the ability for the stiff actuator to tend to remain still with no motor power.

The control method utilized is essentially two-fold. On the one front is the main feedback frequency domain controller with a single feedforward element relative to $X_l$, and on the other hand is a PID controller acting as additional feedback into this system. The input to the controller is the desired force applied to the load, and the output is the actual force applied to the load. The paper then defines some parameters for the PID tuning, given PID(s):

* $PID(s) = K_p + \frac{K_{d}s}{1+\tau_{d}s} + \frac{K_i}{1/\tau_{i}s}$
* $K_p, K_d, K_i$ represent standard PID gains (proportional, derivative, integral)
* $\tau_i$: integral roll-off
* $\tau_d$: derivative roll-off

$Z$ is then defined in terms of the PID term, where PID is on the denominator of this new term. To ensure proper stability, it must be that the PID term has a positive imaginary part. The paper then suggests this is true when $\tau_i \leq \sqrt{frac{K_d}{K_i}}$. This essentially concludes the papers discussion of controller design.

Once implemented, the controller was subjected to some basic performance testing based on means squared error at given frequencies. At low frequencies there were negligible errors, at resonance, the only notable errors occurred close to $Z = 0$, and at high frequencies all notable errors were located where the output impedance has negative real parts - denoting that the controller only works well in this case when acting as a spring or dampener.

### Comments

* The paper does not mention positional control of the load, so it would be interesting to see what sort of added difficulties this task would bring on.
* The paper never mentions compensating for motor saturation, but in the results they claim that errors at resonance were caused by motor saturation. Perhaps they could have mentioned the difficulties in implementing these factors (assuming they are too difficult to practically implement), as they were shown to have non-negligible effects.
* It would be interesting to see more about trajectory planning forces kept in mind, as it would appear these actuators could make this a possibility.

### Related Papers

Some papers such as [this one](https://ieeexplore-ieee-org.ezproxy.uky.edu/stamp/stamp.jsp?tp=&arnumber=9357983) have begun exploring how to implement series elastic actuators in practical environments.
