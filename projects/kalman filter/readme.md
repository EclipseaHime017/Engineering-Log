## Unidimensional Kalman Filter Estimator

In the simulation file(simulation.py), a simple car model is presented with a simple kalman filter estimator of the position. 
By running the program, you will see a blue-square car running along the road under a simple control policy.
The diagram shows the real-time position, the observation and the estimation.
The observer is also provided with very simple choices with unidimensional measurement, though it could be a 2D one.

20/10/2025  
A new G-H-K filter function is implemented. You may switch between kalman filter and g-h-k filter in realization of function ```animate()``` to see different performances.


There are bugs to be fixed.
