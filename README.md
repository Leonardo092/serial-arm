## Serial Arm #
A multi-thread application for Direct and Inverse Geometric Model of a 6-dof serial manipulator.

The main application consists of 3 threads running in parallel and implementing 3 main functions:

- **Producer**: it reads joint values from a csv file (with variable frequency) and performs Direct Geometric Model to find the end-effector pose
- **Consumer**: it acquires the produced pose and performs Inverse Geometric Model to retreive robot joint values
- **Logger**: it reports Producer's and Consumer's outputs on a xml file to generate a joint/end-effector trajectory at fixed frequency (100 Hz)

N.B. Denavit-Hartenberg parameters are hard-coded and calculated according to the most convenient geometry (3 prismatic joints + 3 revolute joints sharing the same origin). Geometrical assumptions are made for the Inverse process as well and reported as comment in the code.


## Package dependencies
- Eigen
- GiNaC

## How to build
Install Eigen3 into your machine.
After cloning, unzip the folder "ginac-1.8.2.zip" and follow instructions from the file "INSTALL" and make sure to install GiNaC as system library.
Navigate to the serial-arm folder and run `$make`to compile the project.
Run `./build/main`to run the application and generate the xml document.
