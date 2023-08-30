This project implements a controller in c++ that aims to maintain the balance of a humanoid robot making non rigid contacts with the environment. Relying on a linearized model of a proposed non-linear model of the environment, and using a trade-off between state and force feedback, an LQR gain is generated, which is used to send acceleration signals to the robots' links making contacts with the environment. This control loop is what I define as a Multi-Soft-Contacts Stabilizer (in short, Msc Stabilizer). The accelerations are sent as tasks to a QP solver that generates the joint positions for the robot. 

In this branch, the controller is implemented on an HRP2KAI robot that makes 3 contacts with the environment: 2 contacts with its feet by standing, and 1 contact with its right hand on a wall. State observation is done using estimators such as the [Kinematic Intertial](https://scaron.info/robot-locomotion/floating-base-estimation.html) and encoders.

The controller is written using the [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/index.html) framework.
