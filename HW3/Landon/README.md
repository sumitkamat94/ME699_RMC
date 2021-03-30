## HW 3 Solution: Landon Clark

* The solution for HW 3 is contained in the julia/odes folder.
* To run the code, `include("startup.jl")` must be run from inside the aforementioned folder.
* From here, `include("panda.jl")` contains the requested solutions to the problems posed.
* Once included, the program will execute the designed PD controller first, simulating a 10 second period of time - calculating the error norm between qa and qd. Once this simulation is completed, the computed torque controller will execute, again with the same simulation features.
* The steady state error for both controllers should be notably under the requested 0.01 norm value, with the PD controller performing as well as ~0.0014 and the CTC achieving ~0.0004. The PD gains were the same on both controllers in order to demonstrate the increased accuracy of the CTC controller.
* As an aside: continuing to increase the Kp values for either controller with some marginal increases to Kd proved to consistently decrease errors, but the values used were indicative of reasonable values of an actual robot manipulator.
