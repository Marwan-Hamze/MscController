This branch tries to implement the kinemtic inertial estimator and fix the controller accordingly so that an experiment on a real robot becomes possible. 

The controller is written using the [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/index.html) framework.

Build and Install using CMAKE.

Run Choreonoid then mc_rtc_ticker display.launch

## GUI use description

Here are quick notes about using the GUI:

1. Go to the "Stabilizer" category. In this branch, please do not use the "FSM" sub-category, which is used for making a contact with the right hand.

2. Start by clicking the "Initialize" button under the "Main" sub-category. This button was meant to set the degrees of freedom of the feet contacts to 0 and make them free to move. This button is needed for the multi-contact case, but here it is not needed and can be removed. The controller starts with the degrees of freedom of the feet contacts equal to 0. 

3. After clicking the "Initialize" button, the "Compute" button will appear under the "Main" sub-category. This button sets the initial configuration of the stabilizer, takes the state of the real robot and uses it as a reference for the stabilizer, and calculates the LQR gain with the linearized matrices. Since the reference is taken as the robot's state, it is recommended to wait for the initial oscillations of the robot to settle down before clicking the button.

4. After clicking the "Compute" button, a set of buttons and sub-categories will appear. Under the "Main" sub-category, buttons to check the current error vectors and other values are provided, in addition to the "Enable" button which loads the contacts and CoM/Base tasks into the QP solver.

5. The sub-categories added are for tuning the Q, R matrices of the LQR and the W matrix for the kinematics-forces trade-off. Note that after modifying the tuning of each matrix, the "Update" button availabe in each of these sub-categories should be clicked for the modifications to take place and to calculate a new LQR gain at the same time.

6. The "Stop" sub-category is to be ignored in this branch, as it is used to fix the feet contacts to the ground in the multi-contact mode.

7. By clicking "Enable", the button is replaced by a "Disable" button that is used to remove the tasks added by the "Enable" button from the QP. When in "Enable" mode, the sub-categories for tuning the Q, R and W matrices are not available. When "Disable" is clicked, the tuning will be available again.

Note: The stiffness/damping of the contacts, the PD gains of the CoM/Base tasks and the admittance gain constant cannot be modified via the GUI, rather the modification of these parameters should be done on the code level. 