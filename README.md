This branch implements the controller on an HRP2KAI robot in a 2 contacts scenario.

Estimation of the floating base is done using the kinematic inertial estimator.

The controller is written using the [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/index.html) framework.

Build and Install using CMAKE.

Run any Choreonoid file using the HRP4 robot then mc_rtc_ticker display.launch. (Running the Choreonoid files installed with this repository might not work)

The default configuration corresponds to the case when the robot is standing on 2 deformable surfaces. The configuration needs to be changed so that the controller works when the robot is standing on the rigid, flat ground.

## GUI use description

Here are quick notes about using the GUI:

1. Go to the "Stabilizer" category.

2. Start by clicking the "Compute" button under the "Initialization" sub-category. This button sets the initial configuration of the stabilizer, takes the state of the real robot and uses it as a reference for the stabilizer, and calculates the LQR gain with the linearized matrices. Since the reference is taken as the robot's state, it is recommended to wait for the initial oscillations of the robot in choreonoid to settle down before clicking the button.

3. After clicking the "Compute" button, a set of buttons and sub-categories will appear. Under the "Main" sub-category, buttons to check the current error vectors and other values are provided, in addition to the "Enable" button which loads the contacts and CoM/Base tasks into the QP solver.

4. The sub-categories added are for tuning the Q, R matrices of the LQR and the W matrix for the kinematics-forces trade-off. Note that after modifying the tuning of each matrix, the "Update" button availabe in each of these sub-categories should be clicked for the modifications to take place and to calculate a new LQR gain at the same time.

5. By clicking "Enable", the button is replaced by a "Disable" button that is used to remove the tasks added by the "Enable" button from the QP. When in "Enable" mode, the sub-categories for tuning the Q, R and W matrices are not available. When "Disable" is clicked, the tuning will be available again.

Note: The stiffness/damping of the contacts, the PD gains of the CoM/Base tasks and the admittance gain constant cannot be modified via the GUI, rather the modification of these parameters should be done on the code level. 