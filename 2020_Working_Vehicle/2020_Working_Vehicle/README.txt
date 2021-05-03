These files were used to fix a problem when running the MOOS simulation where the simulator 
running in pMarineViewer was not properly connecting to the vehicle MOOS Database. 
Simply copy the files into the mission directory on the machine running the simulation.

These files have no effect on the behavior of the autonomous vehicle, they are only to restore
the broken connection when running the simulator. Please note the configuration block in 
meta_vehicle_sim.moos does not include the necessary arguments to use the pBrains app, please 
add the arguments according to the pBrains User Manual in order to automatically run the app
when the simulator starts running.

Credit for these configuration files goes to Matthew S. Woodward, Grad Student, who assisted
the project by providing these files.
