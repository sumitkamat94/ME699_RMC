## HW 3 Solution: Landon Clark

* The solution for HW 3 is contained in the julia/odes folder.
* To run the code, `include("startup.jl")` must be run from inside the aforementioned folder.
* From here, `include("panda.jl")` contains the requested solutions to the problems posed.
* Once included, the program will execute the designed PD controller first, simulating a 20 second period of time - allowing for the steady state response to be observed. Once this simulation is completed, the computed torque controller will execute, again with the same simulation features.
* The steady state error for both controllers should be notably under the requested 0.01 norm value, with the PD controller performing as well as ~0.003 and the CTC achieving ~0.005. The PD feedback gains for the CTC were roughly half of those in the PD controller.
* As an aside on the performance of both: even though the PD controller had better long term performance, it is clear when observing the simulations that the CTC converges much quicker and smoother than its PD counterpart.
