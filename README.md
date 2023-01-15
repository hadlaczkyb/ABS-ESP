# Design of Control Stability Program for Vehicles - BSc Thesis Work
## Abstract

In the recent decades computer science has undergone on a massive evolution.
Thanks to this, the number of comfort and safety features implemented in modern
vehicles has sky-rocketed. The most well-known examples are the ABS (Anti-Lock
Braking System) and the ESP (Electronic Stability Program).

ABS operates by preventing the wheels from locking up, thus ensuring shorter
braking distances on different road conditions, and maintaining the controllability,
stability of the vehicle in case of sudden braking. The main function of the ESP is improving 
the stability of the vehicle and preventing skidding and loss of control during rapid
manoeuvring. Today the ABS can be considered as a standard equipment in new vehicles,
while the ESP is still mostly used in high end luxury vehicles and sports cars.
Both ABS and ESP requires the implementation of controllers with high efficiency.
However due to the high non-linearity of these systems, traditional techniques used for
linear systems cannot be applied.

First, the chosen controller for ABS was the PID. Because of the above mentioned
non-linear behaviour of the system, the controller can be tuned either with genetic
algorithm or with particle swarm optimization technique. In order to ensure the
efficient operation of the system on different road conditions an ANFIS (Adaptive Neuro-Fuzzy Inference System)
road estimator was implemented. It uses the friction coefficient of the wheels to calculate
the desired slip for the current road type. As an addition a user interface was created,
where both the optimization and the simulation can be done. A 3D animation was also made,
which showcases the workings of the system in comparison with the behaviour of a vehicle
without ABS.

Second, a more complex, full vehicle model was designed in order to describe not
only the longitudinal, but the lateral dynamics of the vehicle for different road conditions.
Then an ABS and an ESP controllers were added that work with fuzzy logic. Finally, the
whole model was completed with a 3D visualization to demonstrate the
behaviour of the vehicle with and without the above mentioned safety systems.
