# Self-Driving-Vehicle-Control
This poject is the final project assignment of introduction to Self Driving Cars course offered by University of Toronto on Coursera. The "controller2d.py" file contains a controller object. I implemented the controller in the update_controls method.
[video]
https://www.youtube.com/watch?v=Pu3B4sGw5uc&pbjreload=10
## Self Driving Cars Longitudinal and Lateral Control Design
In this project, I implemented a controller in Python and use it to drive a car autonomously around a track in Carla Simulator.
The output of the controller will be the vehicle throttle, brake and steering angle commands.
The throttle and brake come from the Longitudinal speed control and the steering comes from our Lateral Control.

### 1. Longitudinal Control

![image](https://user-images.githubusercontent.com/59261333/73611832-b9cd6480-45ee-11ea-8b15-58c1068bf7ed.png)

For longitudinal control I implemented a PID Controller, This PID controller will take the desired speed as the reference and outputs throttle and brake.

a PID controller consists of three components. First, a pure gain Kp that scales the vehicle acceleration based on the speed error. This ensures that the vehicle is accelerating in the correct direction with the magnitude proportional to the error.

Second in integral term KI sets up the output based on accumulated past errors. This ensures the steady steed errors are eliminated for ramp referencing.

Finally, the derivative term KD dampens the overshoot caused by the integration term.

To complete the longitudinal control, we must convert the acceleration output from the PID controller into throttle and brake commands. For simplicity, positive outputs will be throttle and negative outputs will correspond to brake.

### PID Controller

The basic idea behind a PID controller is to read a sensor, then compute the desired actuator output by calculating proportional, integral, and derivative responses and summing those three components to compute the output.

![image](https://user-images.githubusercontent.com/59261333/73613652-94495680-4600-11ea-9e6d-124fb5bfc188.png)

#### P - Proportional part
- The proportional part is easiest to understand: The output of the proportional part is the product of gain and measured error e. Hence, larger proportional gain or error makes for greater output from the proportional part. Setting the proportional gain too high causes a controller to repeatedly overshoot the setpoint, leading to oscillation.
- e = r – y
e: error,    r: user input reference,    y: actual measured output
- Proportional part = Kp * e
Kp: input gain for P – Regulator (Proportional Regulator)

```
# Proportional part
Proportional = kp * e_current
```

#### I - Integral part
- Think of the integral  as a basket in which the loop stores all measured errors in E. Remember that error can be positive or negative, so sometimes error fills the basket (when positive error is added to positive error or negative error is added to negative) and sometimes it empties the basket — as when positive error is added to negative, or vice versa.
- collect all of these errors over time (take integral over time).
- Interal part = Ki * E  
  where E = E + e(current) * ∆t,         the loop stores all measured errors in E

```
# integral part
self.vars.E = self.vars.E + e_current * delta_t 
integral = ki * self.vars.E
```
#### D - Derevative part
- the derivative is looking at the rate of change of the error. The more error changes, the larger the derivative factor becomes.
- Kd have to be less than 1.
- Derevative part = Kd * ((e(current) - e(previous)) / ∆t)

```
# derivate part
if delta_t == 0:
  derivate = 0
else:
  derivate = kd * ((e_current - self.vars.e_previous)/delta_t)
```
#### u - System input signal (PID controller output signal)
```
u = Proportional + integral + derivate    # u : input signal

if u >= 0:
  throttle_output = u
  brake_output    = 0
elif u < 0:
  throttle_output = 0
  brake_output    = -u
```

#### pid controller output graph

![image](https://user-images.githubusercontent.com/59261333/73612439-7a097b80-45f4-11ea-8cb6-cf13591116d8.png)

The rise time: the time it takes to reach 90% of the reference value.

Th overshoot: the maximum percentage the output exceeds this reference.

The settling time: the time to settle to within 5% of the reference.

The steady state error: The error between te output and the reference at steady state.

![image](https://user-images.githubusercontent.com/59261333/73613341-3e26e400-45fd-11ea-8985-d059e340796e.png)

### 2. Lateral Control

For lateral control, I implemented the Stanley controller. The Stanley method is the path tracking approach used by Stanford University’s autonomous vehicle entry in the DARPA Grand Challenge, Stanley. The Stanley method is a nonlinear feedback function of the cross track error, measured from the center of the front axle to the nearest path point.

![image](https://user-images.githubusercontent.com/59261333/73640204-833b2c80-4676-11ea-8620-077d8c2961f4.png)

- e: Cross track error
- θ_c: Heading of the vehicle (Yaw).
- ψ + θ_c: Heading of the path (path Yaw).
- ψ: Heading error.
- δ: Stearing angle.
- v: Current forward speed (meters per second).
- x_c: Current X position (meters)
- y_c: Current Y position (meters)
##### The resulting steering control law is given as
![image](https://user-images.githubusercontent.com/59261333/73640845-a9150100-4677-11ea-8c9c-4e0e5f906bec.png)

When e is non-zero, the second term adjusts δ such that the intended trajectory intersects the path tangent at kv(t) units from the front axle.

