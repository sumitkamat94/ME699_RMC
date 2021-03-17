@def class = "conference"
@def authors = "Marco, A.; Hennig, P.; Bohg, J.; Schaal S.; Trimpe S.;"
@def title = "Automatic LQR tuning based on Gaussian process global optimization"
@def venue = "2016 IEEE International Conference on Robotics and Automation (ICRA)"
@def year = "2016"
@def hasmath = true
@def review = true
@def tags = ["reviews","learning","control","analysis","optimization"]
@def reviewers = ["Sumit Suryakant Kamat"]

\toc
### Overview / Broad Area
Developing an accurate mathematical model of a physical dynamic system can be challenging, time-intensive, and expensive. Furthermore, the mathematical model of the physical dynamic system may not account for parametric uncertainties, non-linearities, unmodeled dynamics, and disturbances due to the limitations of the sensors. If a controller is tuned without considering model uncertainties, then the controller might not achieve the required performance. This motivates the tuning of controller gains using experimental data, as it would reduce the effects of the above-mentioned uncertainties. 

However, tuning of controller gains using methods such as grid search is time-intensive and might require multiple experimental. This necessitates the development of efficient algorithms for obtaining controller gains.

This paper proposes a framework for tuning controller gains by combining optimal control with Bayesian optimization using limited experimental data. 

[//]: # (Bayesian Optimization involves solving optimization problems where the objective function is continuous but does not have a special structure for e.g.; concavity. Furthermore, the objective function does not give gradient information.)

### Notation
* $x_k$: discrete state, $\tilde{x}_k$: approximate discrete state
* $A_{\text{n}}$, $B_{\text{n}}$: Nominal plant parameters
* $u_k$: Control, $w_k$: noise
* $J$: Nominal Quadratic cost function
* $\hat{J}$: Approximate cost function
* $\mathcal{D} \sub \mathbb{R}^D$: Doman representing region around nominal design


### Problem Statement
Consider the non-linear discrete model $x_{k+1}=f(x_k,u_k,w_k)$, where $x_k \in \mathbb{R}^{n_x}$ and $u_k \in \mathbb{R}^{n_u}$ is the control. The non-linear discrete model has an equilibrium at $x_k=0$, $u_k=0$, and $w_k=0$. The non-linear system can be approximated as a linear discrete (nominal) model $\tilde{x}_{k+1}=A_n \tilde{x}_{k} +B_n u_k+w_k$ about the equilibirirum. 

The nominal cost function is given by $J=\lim_{K \to \infty} \frac{1}{K}\mathbb{E} [\sum_{k=0}^{K-1}x_k^TQx_k+u_k^TRu_k]$, where $Q$ and $R$ are positive-definite weigthing matrices. The objective is to determine the optimal control method which would minimize the cost function, while using data efficiently from fewer experiments. 

### Solution 
 
First, Marco et al. determine the state-feedback control for the non-linear system which would minimize the cost function $J$.
* LQR control given by $u_k=Fx_k$ is a good candidate for minimizing the nominal cost function $J$; where $F=\text{lqr} (A_{n},B_{n},Q,R)$. 
* The controller gain can be parameterized as $F=\text{lqr} (A_{n},B_{n},W_x(\theta),W_u(\theta))$. This is done to accoun
 It follows that $J=J(\theta)$.  

* The objective is to minimize $J(\theta)$ by varying the parameters $\theta$. In particular, the optimization problem is given by  

$$\text{arg} \ \text{min} \ J(\theta) \ \ \text{s.t.} \ \ \theta \in \mathcal{D}.$$

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In other words, we minimize $J$, where $\theta$ is restricted to $\mathcal{D}$. 

* However, note that the cost function $J$ cannot be determined from the experimental data. This is because $J$ represents an infinite horizon problem. Hence, they consider the approximate cost function $\hat{J}= \frac{1}{K} [\sum_{k=0}^{K-1}x_k^TQx_k+u_k^TRu_k]$.


Next, Entropy Search (ES) is combined with optimal control to determine the parameters $\theta$, which is summarized as follows:
* The uncertainity over the objective function $J$ is represented as a probablity measure $p(J)$.
* The prior knowledge of the objection function can be modeled as a Gaussian process, $J(\theta) \sim \mathcal{GP}(\mu(\theta), k(\theta, \theta_*))$, where $\mu(\theta)$ is the mean function and $k(\theta,\theta_*)$ is a covariance function.
* Note that approximate cost function $\hat{J}$ can be considered as a noisy evaluation of $J(\theta)$. Thus we can model the approximate cost function as $\hat{J}=J(\theta)+\epsilon$, where $\epsilon$ represents the gaussian noise with variance $\sigma_{\text{n}}^2$.
* The optimization problem is defined as, $p_{\text{min}}(\theta)=p(\theta=\text{arg} \ \text{min} \ J(\theta)) \ \ \text{s.t.} \ \ \theta \in \mathcal{D}.$


* ES is used to suggest the location of the parameter $\theta$ where the approximate cost function $\hat{J}$ is minimzed. This is done by selecting the next evaluation point which maximizes the relative entropy $H=\int_{\mathcal{D}} p_{\text{min}}(\theta) \text{log} \frac{p_\text{min}(\theta)}{b(\theta)} \text{d} \theta$, between $p_{min} (\theta)$ and the uniform distribution $b(\theta)$ over the bounded region $\mathcal{D}$. 

* ES selects the next parameter for which the approximate cost function is evaluated by determining where the maximal change of entorpy $\Delta H(\theta)$ is pretty high.

* ES also returns the best guess of the maximum of the approximation of $p_{\text{min}}(\theta)$.

The above mentioned methods when combined together form the Automatic LQR tuning method. The algorithm of the LQR tuning method can be summarized as follows:
* The approximate cost ($\hat{J}$) is evaluated initially with $Q$ and $R$. 
* Then ES is used to return the best guess of $\theta$ at which we minimize $\hat{J}$.
* The approximate cost is evaluated using the weighting matrices $(W_x(\theta), W_u(\theta))$.
* We then repeat the above two steps until not much improvement is noticed in the approximate cost function.

The paper also discusses experimental results involving a one degree of freedom balancing problem, where they succeessfully demonstrate the tuning of the LQR gains using the Automatic LQR tuning method. The experiment involves the balancing of a pole which is attatched to the end-effector of a robot. They were successful in tuning up to 4 parameters using Automatic LQR tuning in the prescence of uncertainties.

### Comments
* The paper focuses on LQR tuning using ES.
* Instead of determining both weighting matrices $Q$ and $R$ we parameterize the weighting matrices as a function of $\theta$ and determine $\theta$.
* Entropy search is used to determine the best guess of the location of the parameter $\theta$, which in turn is used to evaluate the approximate cost function $\hat{J}$. 
* Other Bayesian algorithms do not store information about the optimal location, however ES aims at collecting the information of the optimal location.
* Automatic LQR tuning is good to develop appropriate final designs, as this method not only determines the controller gains, but also determines the information of the optimal location.
* The ES algorithm has higher computational cost in comparison to other Gaussian Process methods, as we don't prioritize information gain in the other methods.
* The ES algorithm yields improved controllers much faster than alternative methods where we want to maximize information gain ([P. Hennig 2012](https://jmlr.csail.mit.edu/papers/volume13/hennig12a/hennig12a.pdf)).
* However, this automatic tuning framework has yet to be further evaluated in practice.