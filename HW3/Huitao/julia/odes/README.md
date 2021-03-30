### Instructions
1. type 'include("Assignment3.jl")' to run the file, norm errors for both controller are less than 0.01.
2. Method to get joint traj: cubic interpolation.
3. To change between PD control and CTC control, just simply change the control part in the 'problem' at the line: 73. Use _**control_CTC!**_ or  _**control_PD!**_
4. The P value under _**control_CTC!**_ shows how small we can get the Kp with respect to _**control_PD!**_, P=0.4 is the proportion with an error less than 0.01. _Note_: the Kd in the _**control_CTC!**_ is 10 time larger than _**control_PD!**_ to eliminate the oscilation.
