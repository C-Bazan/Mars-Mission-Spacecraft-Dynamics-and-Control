# Mars-Mission-Spacecraft-Dynamics-and-Control
Capstone project for the Spacecraft Dynamics and Control Specialization offered by the University of Boulder, Colorado via Coursera

This project is about the simulation of a Mars Mission that consists of the attitude dynamics and control of a nano-satellite orbiting around Mars in Low Orbit. The mission consists of controlling the attitude of the spacecraft with a proportional derivative controller in 3 scenarios: 

1. Power Mode, where the solar panels of the satellite must be pointing towards the sun. 

2. Science Mode, where the sensors of the satellite must be pointing towards the planet.

3. Communication Mode, where the satellite must point its communication antenna towards a mothercraft that is in Geosynchronous Mars Orbit, whenever the mothercraft is visible and the satellite is not on the sunlit side of Mars.

# Code Sample

A code sample of the orbital and attitude propagator is included in the repository. The code sample shows the core logic of the orbital and attitude propagator, including the switching between Power, Science, and Communication modes, and the integration of attitude dynamics using a Runge-Kutta method.

# Animation

The animation of the simulation is available in the following link: https://drive.google.com/file/d/1Pn7tv1U42O4RAZraI5wqekroBN6khlJC/view?usp=sharing
The animation demonstrates the satellite switching between the three attitude modes during a simulated Mars orbit, showing how the control law stabilizes the satellite while maintaining the mission objectives.

# Plots of the Parameters

The following plots show the evolution of the modified Rodrigues parameters (σ B/N) and the satellite's angular rates (ω B/N) throughout the simulation.

<img width="1536" height="767" alt="Figure_1" src="https://github.com/user-attachments/assets/abd13ad9-3ebe-488f-a68b-da6c933c078c" />
<img width="1536" height="767" alt="Figure_2" src="https://github.com/user-attachments/assets/3f07f118-f5d0-4134-b96f-a845179436ad" />
