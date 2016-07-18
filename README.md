# LightningBolt---PID-enabled-fast-line-follower
Sensors Used-
 Pololu QTR Analog sensor 

a fast line follower base on proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism (controller) commonly used in industrial control systems. A PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable. The controller attempts to minimize the error over time by adjustment of a control variable, such as the position of a control valve, a damper, or the power supplied to a heating element, to a new value determined by a weighted sum:

    u ( t ) = K p * e ( t ) + K i* ∫ e ( τ ) d τ + K d * d e ( t )/d t 

where K p {\displaystyle K_{p}} K_{p}, K i {\displaystyle K_{i}} K_{i}, and K d {\displaystyle K_{d}} K_{d}, all non-negative, denote the coefficients for the proportional, integral, and derivative terms, respectively (sometimes denoted P, I, and D). In this model,
P accounts for present values of the error. For example, if the error is large and positive, the control output will also be large and positive.
I accounts for past values of the error. For example, if the current output is not sufficiently strong, error will accumulate over time, and the controller will respond by applying a stronger action.
D accounts for possible future values of the error, based on its current rate of change


 
