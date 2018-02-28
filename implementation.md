# The Model

The project is an implementation of Model Predictive Control on a Global Kinematic Model. A cubic polynomial trajectory is fitted to the centre of the lane. MPC is used alongside this reference trajectory to autonomously drive the vehicle. 

The properties of the Global Kinematic Model are described below:

## States
The state of the model consists of:
* x - x position of the vehicle in the world
* y - y position of the vehicle in the world
* psi - orientation of the vehicle in the world
* v - velocity of the vehicle

and two derived state elements based on the reference trajectory:
* cte - cross track error between vehicle and reference trajectory (fitted polynomial to centre of the lane)
* epsi - difference between vehicle's orientation and trajectory orientation

## Actuators 
The vehicle has two actuators, steering (delta) and throttle (a).

## Update equations

	x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
	y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
	psi[t+1] = psi[t] - (v[t] / Lf) * delta[t] * dt
	v[t+1] = v[t] + a[t] * dt
	cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
	epsi[t+1] = psi[t] - psides[t] - (v[t] / Lf) * delta[t] * dt

Where f() is the cubic polynomial representation of the reference trajectory, Lf is the distance between the vehicle's steering and centre of gravity, delta is the steering and a is the actuator input. 

## Reference Trajectory

A cubic polynomial is fitted to the centre of the lane. The simulator provides a series of waypoints representing sample points from the centre of the road in the front of the vehicle. We first transform these waypoints to the vehicle's coordinate system before fitting a cubic polynomial to these points. The transformation used is:

	x_t = x * cos(-psi) - y * sin(-psi)
	y_t = y * sin(-psi) + y * cos(-psi)

## Timestep length and elapsed duration (N & dt)

A longer time duration was initially tested (N = 20, dt = 0.1), however results were poor. It's likely this is because the polynomial reference trajectory is less accurate when considering waypoints further away (a cubic polynomial would only take into account one or two curves well). Thus, to take this into account when performing MPC would contain constraints on less accurate data (cte and epsi on a relatively less accurate and less important portion of the road). 

Thus the time duration was decreased. (N = 10, dt = 0.05). This shorter duration and more granular allowed the optimiser to implicitly focus on the portion of road closer to the vehicle. A shorter time interval (dt = 0.05) was used to give the vehicle better control in light of added latency. 

## Handling latency

In order to handle the 100 millisecond latency, the previous steering and throttle values were saved and used to constrain the first 100 milliseconds of predictions (for dt = 0.05, the first 2 values for steering and throttle actuation are fixed to the previous steering and throttle value respectively). This would assume the controller has no control in the first 100 milliseconds of latency and thus can only begin to optimise once this latency period is over. 


