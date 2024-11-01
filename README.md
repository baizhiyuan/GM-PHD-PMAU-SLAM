The project contains an implementation of GM-PHD-PMAU-SLAM [1], which was developed by building upon the open-source contributions of PHD-SLAM 2.0 [2] and PHD-SLAM 3.0 [3]. This approach provides a pose and map alternating estimation mechanism to enhance the performance of PHD-SLAM in complex environments. Unlike the previous versions, our implementation employs the Unscented Kalman Filter (UKF) instead of the Particle Filter (PF), which significantly improves computational efficiency. Moreover, the accuracy and robustness are further enhanced through Covariance Intersection (CI) fusion. Additionally, a labeled version of the algorithm, LGM-PHD-PMAU-SLAM, is provided, which utilizes label information for relocalization to reduce accumulated errors. The details of the algorithm are presented in the paper "A GM-PHD-SLAM Algorithm Based on Pose and Map Alternating Update" [1].

The GM-PHD-PMAU-SLAM implementation was developed within the [PHD-SLAM-3.0](https://github.com/okaltiok/PHD-SLAM-3.0) framework, ensuring that usage remains as consistent as possible with the original version to facilitate performance analysis and comparison. We would like to express our gratitude to Ossi Kaltiokallio and collaborators for their open-source contributions. Below are the steps for using the code:

1. Download the project to your computer.

2. GM-PHD-PMAU-SLAM with synthetic data.\
    a) Open file ".\GM-PHD-PMAU-SLAM\Synthetic Data\MonteCarloSimulations.m"\
    b) Set option = 1 and run the code to create the data.\
    c) Set option = 2 and run the code to perform the Monte Carlo simulations (MCSs). This will take some time since it        will perform 100 MCS with three algorithms. You should get the following results:
    
       GM-PHD-PMAU-SLAM Performance:
       N:1,   POS.std.=5.34±3.39 [m], HEAD.std.=0.66±0.36 [deg], GOSPA std.=59.12±30.71 [m], 
              CPU std.=0.79±0.06 [ms], TIME=3.16 [s]
       LGM-PHD-PMAU-SLAM Performance:
       N:1,   POS.std.=4.08±2.04 [m], HEAD.std.=0.52±0.17 [deg], GOSPA std.=53.68±24.16 [m], 
              CPU std.=0.67±0.03 [ms], TIME=2.67 [s]
       PHD-SLAM 3.0 Performance:
       N:1,   POS.std.=6.76±3.43 [m], HEAD.std.=0.80±0.29 [deg], GOSPA std.=80.94±39.05 [m], 
              CPU std.=0.24±0.04 [ms], TIME=0.98 [s]
       N:5,   POS.std.=5.92±2.92 [m], HEAD.std.=0.72±0.24 [deg], GOSPA std.=72.26±34.05 [m], 
              CPU std.=1.03±0.07 [ms], TIME=4.12 [s]
       N:30,  POS.std.=5.54±2.86 [m], HEAD.std.=0.69±0.22 [deg], GOSPA std.=66.38±34.82 [m], 
              CPU std.=5.00±0.08 [ms], TIME=20.02 [s]
    
    d) The results are stored in the results-folder, and the performance metrics can be computed by setting option = 3         and running the code. 
    
3. GM-PHD-PMAU-SLAM with Victoria Park data set.\
    a) Download "victoria_park.zip" from [4] and unzip the folder in to "Victoria Park/measurements/VP data set"\
    b) Open file ".\GM-PHD-PMAU-SLAM\Victoria Park\MonteCarloSimulations.m"\
    c) Set option = 1 and run the code to create the data.\
    d) Set option = 2 and run the code to perform MCSs. You should get the following result:
    
       GM-PHD-PMAU-SLAM Performance:
       N:1,  POS.=2.55 [m], CPU=2.93 [ms], TIME=21.18 [s]
       LGM-PHD-PMAU-SLAM Performance:
       N:1,  POS.=2.46 [m], CPU=3.09 [ms], TIME=22.33 [s]
    
    e) The results are stored in the results-folder, and the performance metrics can be computed by setting option = 3         and running the code. 
    
4. If Matlab throws an error from the mex-files, you need to compile the mex-files on your computer. The mex-files can be compiled using CompileCLibraries.m found in the folder ".\GM-PHD-PMAU-SLAM\Shared Files\mex source". Please note that the mex-files have to be compiled seperately for both data sets. Further instructions can be found in CompileCLibraries.m.

5. The function perform_mcs() in MonteCarloSimulations.m utilizes Matlab's Parallel Computing Toolbox to run the simulations with multiple CPU cores in parallel. If you don't have Parallel Computing Toolbox installed, simply change the parfor-loop to a regular for-loop.

6. Please note that, as the paper is currently under review, the present version of the code is a preview. It can produce experimental results using the mex-files but does not yet include the full internal implementation of the algorithm. The complete code will be released after the paper is published.


References:

[1] H. Shentu, Z. Bai et al., "A GM-PHD-SLAM Algorithm Based on Pose and Map Alternating Update," submitted.

[2] L. Gao, G. Battistelli and L. Chisci, "PHD-SLAM 2.0: Efficient SLAM in the Presence of Missdetections and Clutter," in IEEE Transactions on Robotics, doi: 10.1109/TRO.2021.3052078.

[3] O. Kaltiokallio et al., "A Multi-Hypotheses Importance Density for SLAM in Cluttered Scenarios," in IEEE Transactions on Robotics, doi: 10.1109/TRO.2023.3338975

[4] “Victoria park SLAM data set,” Jose Guivant, Australian Centre for Field Robotics - The University of Sydney.\ Accessed Dec. 21, 2023. [Online]. Available: http://www-personal.acfr.usyd.edu.au/nebot/victoria_park.htm
