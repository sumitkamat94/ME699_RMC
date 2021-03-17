@def class = "journal"
@def authors = "Panagiotis K. Artemiadis, Pantelis T. Kastsiaris, Kostas J. Kyriakopoulos"
@def title = "A Biomimetic Approach to Inverse Kinematics for a redundant Robot Arm"
@def venue = "Springer Sciense"
@def year = "2009"
@def hasmath = true
@def review = true
@def tags = ["Human-like robot arm","graphical model","Baysian framework"]
@def reviewers = ["Guan Huitao","Sumit Kamat","Landon Clark"]

\toc
### Broad area/overview
This paper solved the redundancy problem of human-like robotic arm with a biomimetic approach. The kinematics and configuration of upper arm was modeled and investigated. The data was collected from real human subjects. Then, the motion data was used to build a graphical tree model(with nodes and edges) and pluged in to a Gaussian Mixture Model(GMM), which give us a way to optimize the angles of the joints with given root joint angle. The main novelty of this method is the method is not limited to a finite set of human motion data and the resulting robot arm motion is not restricted to identically mimic human motions. Instead, it has the ability to generate new motions.

### Notation
* $q$: joint angle
* $x,y,z$: the relative location from sensor to base sensor
* $I$: mutual information
* $g$: probability density function(PDF) of a GMM
* $J$: Jacobian matrices

### Specific Problem
* Instead of mimicking the human motion, this study focuses on looking for a relationship between the joints. There were 4 human subjects participated in the study, their random upper arm motion was collected with respect to the angles of each joints. Then the sutdy becomes to find the dependencies of each joints with Baysian frame work. Moreover, by fitting the frame work to GMM we can conclude to the continuous representation. Finally, to find a most human like inverse kinematics solution boils down to maximize the Gaussian function G(q) in the null space.

### Solution Ideas
* Firstly, 5 DOF upper arm model was selected with: 3DOF shoulder joint, 1DOF elbow joint and 1DOF forearm pronation-supination. The subjects have 3 sensors attached: one base sensor at shoulder, two sensors at elbow and wrist. The position data (XYZ) was then converted to joint angles with inverse kinematics methods
* To capture the conditional dependency of each joint, a acyclic graphical model was built. The model edges was selected with respect to mutual information. The joint angles were discretized to nearest intger for simplification of the graphical model training.
* For each joint angle the dependency is concluded to a continuous representation, by fitting with GMM. The resulting g(i,j) is the PDF of conditional probability distribution funtion of joint angle i given its parent joint angle j.
* The g(i,j) is based on joint and marginal probabilities, it is maximized  if the corresponding coordination is frequently observed druing the human motion.
* Given root joint angle, we can maximize the parent joints' angle then the children joint anlge. Therefore, we can have the human-like inverse kinematics solution with any given root joint angles.



### Comments
 * This paper train a graphical model of dependencies of each upper arm joint angles, then the PDF of each correlated joint was fit into GMM to get continuous representation. The peak value of g was then obtained and set to be the most human-like angle, because it is the most frequently observed in the experiment.
 * This method can generate new motions and independent from pre-existing data.
 * Note that the method only consider 5DOF while we commonly use 7DOF.
 * Note that the method heavily depend on the given root joint angles.
