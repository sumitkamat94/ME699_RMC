@def class = "journal"
@def authors = "Shiqiu Gong, Biyun Xie;"
@def title = "Task Motion Planning for Anthropomorphic Arms Based on Human Arm Movement Primitives"
@def venue = "Industrial Robot"
@def year = "2020"
@def hasmath = true
@def review = true
@def tags = ["Human Arm Movement Primitives","Task Planning","Motion Planning","human-like robot arms"]
@def reviewers = ["Guan Huitao", "Sumit Kamat"]

\toc
### Broad area/overview
* This paper represents a set of novel Human arm movement primitives, along with task slicing and sequencing; human arm movement primitives slicing and sequencing.

### Notation
* $\hat{Φ}$: rotation along the line between shoulder joint center and wrist joint center.(swivel angle)
* $\hat{P}$: position change
* $\hat{O}$: orientation change
* $\hat{F}$: force change

### Specific Problem
* Achieve the human-like motion and solve for the redundancy problem, we use four parameters to discribe the human arm motion, the specific combination of the four parameters can be used to define all human arm motions.
* Any task can be decomposed to subtasks and any subtask can be decomposed to HAMPs, the HAMPs then are mapped to joint angles by analytical inverse kinematics.


### Solution Ideas
* For any given task, firstly, slice the task into several coretasks and subtasks. The coretasks are the task of perform the desired goal, i.e, move a cup from A to B, and the subtasks are the prepare motion, i.e, move from the fianl position of last coretask to the initial position of next coretask. The subtasks were then sequenced by Monte Carlo mehtod.
* To execute the subtasks, HAMPs need to be selected and sequenced. Note that four parameters can have 16 different combinations with one of them is null. For this purpose, a total of 12 huamn subjects performed human arm movement for reaching and gripping. Due to the results of the experiment, we have selected the (ΦPO),(PO),(P) to be the HAMPs that we use and also the exact sequence we follow.
* The paper quantified the HAMPs using gaussian and cosine bell-shaped function, because it is widely accepted that the velocity of a single physiological joint and the linear velocity of human arm end-point are unimodal bell-shaped. The mapping can be expressed as:

θ<sub>i</sub> = f(Φ(t),P(t),O(t))

P(t) = P<sub>0</sub>+∫P(t)dt

Φ(t) = Φ<sub>0</sub>+∫Φ(t)dt

O(t) = O<sub>0</sub>+∫O(t)dt

where the P(t),Φ(t),O(t) can be obtained by the bell-shaped function.
* The HAMPs can then be applied to the inverse kinematics, then the general representation of each joint angle can be obtained.

### Comments
* The represented HAMPs are novel and can fully express the movements of human arm and Anthropomorphic arm, which constitude a movement primitives library that can be applied to task motion planning.
* The HAMPs seclecting and sequencing are based on the human subjects motion capture data, thus it is similar to real human arm motions.
* One interesting finding of this study is that the HAMPs were applied to the bell-shaped function, the resulting angle velocities of each joint are also bell-shaped. The characteristic of P(t), Φ(t), and O(t) mapped to the joint angles, which inspires us to use bell-shaped velocities of joint to achieve human-like motions for anthropomorphic arm without human-like indicators.

### Related Paper
Zhao, J., Gong, S. and Zhang, Z. (2018), “Analytical inverse
kinematics of anthropomorphic movements for 7-DOF
humanoid manipulators”, Journal of Mechanical Engineering,
Vol. 54 No. 21, doi: 10.3901/JME.2018.21.025.
