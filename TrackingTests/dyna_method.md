## Guide to running DynaTracker
- Before running this script, ensure that the Dynamixel servo is connected to the computer via USB and that the QTM mocap system is running and streaming data.

- In QTM make sure to align the local coordinate system; this can be done by going into the project options and selecting the 6DOF tracking; then select the rigid body and align the local coordinate system correctly between the points.

- Yaw tracking points local x axis at the target, pitch tracking points local z axis at the target; this will depend on orientation of the rigid body in QTM.

- The working implementation uses the x-axis pointing at the target during the inital phase of the code running.

### Todo:
- [x] Add in the ability to change the tracking axis
- [x] Further investigate 6DOF rigid body placement with regards to the optical axis; as long as the distance and rotations between the body are known, the tracking can be mapped to optical axis.
- [x] Implement more exception handling for initialisation of the Dynamixel servo ie. if the servo is not connected to the computer, the code will not run and will prompt user and wait.