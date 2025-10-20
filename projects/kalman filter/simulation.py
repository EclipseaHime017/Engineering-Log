import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

class Simulator:
    def __init__(self):
        #user configration
        self.dt = 0.1  # Time step
        self.total_time = 20  # Total simulation time
        self.car = self.Car(self)
        self.environment = self.Environment(self)
        self.observer = self.Observer(self, sensor_type="Lidar")
        self.estimator = self.Estimator(simulator=self, Object=self.car, Observer=self.observer)

        # Simulation parameters
        self.time_steps = int(self.total_time / self.dt)
        self.time = np.arange(0, self.total_time, self.dt)
        self.time_index = 0

    class Car:
        def __init__(self, simulator):
            self.simulator = simulator
            self.position = 0.0  # Initial position
            self.velocity = 0.0  # Initial velocity
            self.acceleration = 0.0  # Initial acceleration
            self.length = 4.5  # Length of the car
            self.width = 2.0  # Width of the car
            self.color = 'blue'  # Color of the car

            self.process_noise_std = 1

        def control(self, t: float):
            if t < 10:
                return 1.5  # Accelerate
            elif t < 15:
                return 0.0  # Decelerate
            else:
                return -3.0  # Constant speed
            
        def update(self, acceleration, dt):
            self.position += self.velocity * dt
            self.velocity += self.acceleration * dt
            self.acceleration = acceleration + np.random.normal(0, self.process_noise_std) # add process noise
      
    class Environment:
        def __init__(self, simulator):
            self.simulator = simulator
            self.road_length = 200.0  # Length of the road
            self.road_width = 10.0  # Width of the road
        
    class Observer:
        """ This is an observer including sensor information """
        def __init__(self, simulator, sensor_type="Lidar"):
            self.simulator = simulator
            self.sensor_type = sensor_type
            if sensor_type == "Lidar":
                self.measurement_noise_std = 10
            elif sensor_type == "Satellite":
                self.measurement_noise_std = 1
            elif sensor_type == "Accelerometer":
                self.measurement_noise_std = 0.05
            else:
                raise ValueError("Unsupported sensor type")

        def measure(self, car): 
            """ 
            Measure the car's state with noise.
            Since this is a simple simulation, 
            we assume the sensor can only measure one of the car's attributes based on the sensor type.
            i.e. H = Indentity matrix
            """
            noise = np.random.normal(0, self.measurement_noise_std)
            if self.sensor_type == "Lidar":
                return car.position + noise
            elif self.sensor_type == "Satellite":
                return car.velocity + noise
            elif self.sensor_type == "Accelerometer":
                return car.acceleration + noise
            else:
                raise Exception("Invalid measurement")
            
    class Estimator:
        """
        This is an estimator based on the Kalman filter and simple car model
        """
        def __init__(self, simulator, Object=None, Observer=None):
            self.simulator = simulator
            self.dt = simulator.dt
            if Object is not None and Observer is not None:
                self.object = Object
                self.observer = Observer
            else:
                raise Exception("Invalid measurement")
            
            # Model Parameters Initialization
            self.F = np.array([[1, self.dt],
                               [0, 1]])                         # State transition model
            self.G = np.array([[0.5 * self.dt ** 2],
                               [self.dt]])                      # Control input model
            self.H = np.array([[1, 0]])                         # Observation model

            observation_noise_var = self.observer.measurement_noise_std ** 2
            process_noise_var = self.object.process_noise_std ** 2

            # Kalman Filter Parameters Initialization
            self.Q = np.array([[process_noise_var, 0],
                               [0, process_noise_var]])         # Process noise covariance
            self.R = np.array([[observation_noise_var]])        # Measurement noise covariance
            self.prior_error = np.array([[0],[0]])              # prior estimate error
            self.posterior_error = np.array([[0],[0]])          # posterior estimate error
            self.posterior_error_cov = np.eye(2)                # posterior estimate error covariance

            # G-H Filter Parameters Initialization
            self.g = 0.2                                        # Gain for position
            self.h = 0.1                                        # Gain for velocity
            self.k = 0.01                                       # Gain for acceleration
            
            # Initial Estimate
            self.x_estimate = np.array([[0],
                                       [0]])                    # state estimate
            
        def g_h_k_filter(self, x_t, v_t, u):
            """
            A simple g-h filter implementation for position and velocity estimation.
            args:
                x_t: position at time t
                v_t: velocity at time t
                u: control input (acceleration)
            Remark 
                prediction:
                    x_t = x_(t-1) + v_(t-1)*dt + 0.5*a_(t-1)*dt^2
                    v_t = v_(t-1) + a_(t-1)*dt
                    a_t = u_t
                update:
                    x_t' = x_t + g*(z_t - x_t)
                    v_t' = v_t + (h/dt)*(z_t - x_t)
                    a_t' = a_t + (k/dt)*(z_t - x_t)
                in which 
                    z_t is the measurement at time t
                    i_t' for i in {x,v,a} is the estimated state after update
            Here, under the circumstance that control input u is known, we directly use u as a_t, which will be more accurate.
            However, u comes with noise in real world and hence estimating a_t is still necessary. 
            When u is unknown, we can estimate a_t as well.
            """
            dt = self.dt
            g = self.g
            h = self.h
            k = self.k
            measurement = self.observer.measure(self.object)
            # Prediction step
            x_t = x_t + v_t * dt + 0.5 * u * (dt ** 2)
            v_t = v_t + u * dt
            a_t = u

            # Update step
            residual = measurement - x_t
            x_t = x_t + g * residual
            v_t = v_t + (h / dt) * residual
            a_t = a_t + (2 * k / (dt ** 2)) * residual

            return x_t, v_t, a_t
       
        def kalman_filter(self,u):
            """
            This function predicts the next state
            
            Remark
                x_t' = F*x_(t-1) + G*u_t + w_t;
                    x_t': the prediction of the current state,
                    x_(t-1): the estimation of the previous state,
                    u_t: the control input
                    w_t: process noise
                x_t = x_t' + K*(z_t - H*x_t')
                    z_t: the measurement/observation at time t, in which z_t = H*x_t + v_t
                    (z_t - H*x_t'): the residual, error between the measurement and prediction
                    K: kalman gain
                e_t = e_t' - K*H*e_t'- K*v = (I - K*H)*e_t' - K*v
                    e_t: posterior estimate error
                    e_t': prior estimate error
                P_t = E[e_t' * e_t'^T] = E[[(I - K*H)*e_t' - K*v] * [(I - K*H)*e_t' - K*v]^T]
                P_t = (I - K*H)*(P_t')*(I - K*H)^T + K*R*K^T
                    P_t: posterior estimate error covariance, used 
                    P_t' = E[e_t' * e_t'^T] : prior estimate error covariance
                    R: measurement noise covariance E[v*v^T]
                P_t = P_t' - K*H*P_t' - P_t'*H^T*K^T + K*H*P_t'*H^T*K^T + K*R*K^T
                By P_t' autocorrelated and P_t' = (P_t')^T: P_t = P_t' - K*H*P_t' - (K*H*P_t')^T + K*(H*P_t'*H^T + R)*K^T
                    loss function: tr(P_t) = tr(P_t') - 2*tr(K*H*P_t') + tr(K*(H*P_t'*H^T + R)*K^T) -> a function of K
                    minimize by derivative: d(tr(P_t))/dK = -2*H*P_t' + 2*K*(H*P_t'*H^T + R) = 0
                K = P_t'*H^T*(H*P_t'*H^T + R)^(-1)

                P_t = (I - K*H)*P_t'
                
                e_t' = F * e_(t-1) + w_t
                P_t' = F * P_(t-1) * F^T + Q
            """
            state = np.array([[self.object.position],
                              [self.object.velocity]])
            x_t_pred = self.F @ state + self.G * u

            # update prior error covariance matrix based on previous posterior error covariance matrix
            P_t_prior = self.F @ self.posterior_error_cov @ self.F.T + self.Q
            # compute Kalman gain
            K = P_t_prior @ self.H.T @ np.linalg.inv(self.H @ P_t_prior @ self.H.T + self.R)
            # update posterior error covariance matrix
            P_t = P_t_prior - K @ self.H @ P_t_prior
            # compute current state estimate
            x_t = x_t_pred + K @ (self.observer.measure(self.object) - self.H @ x_t_pred)

            return x_t, P_t
        
        def update(self, control, x_t, type='Kalman Filter'):
            if type == 'Kalman Filter':
                x_t, P_t = self.kalman_filter(control)
                self.x_estimate = x_t
                self.posterior_error_cov = P_t
            elif type == 'G-H-K Filter':
                x_t, v_t, a_t = self.g_h_k_filter(x_t[0, 0], x_t[1, 0], control)
                self.x_estimate = np.array([[x_t],
                                            [v_t]])
                # a_t is discarded here
            else:
                raise ValueError("Unsupported filter type")

    def visualize(self):
        true_positions = []
        measured_positions = []
        estimated_positions = [] 
        times = []
        
        fig, (ax_road, ax_data) = plt.subplots(2, 1, figsize=(12, 8))
        ax_road.set_xlim(0, self.environment.road_length)
        ax_road.set_ylim(-self.environment.road_width/2, self.environment.road_width/2)
        ax_road.set_xlabel('Position (m)')
        ax_road.set_title('Car Motion on Road')
        ax_road.set_aspect('equal') 
        
        road = Rectangle((0, -self.environment.road_width/2),
                        self.environment.road_length,
                        self.environment.road_width,
                        color='gray', alpha=0.3)
        ax_road.add_patch(road)
        
        car_patch = Rectangle((self.car.position, -self.car.width/2),
                            self.car.length,
                            self.car.width,
                            color=self.car.color)
        ax_road.add_patch(car_patch)
        
        ax_data.set_xlim(0, self.total_time)
        ax_data.set_ylim(0, self.environment.road_length)
        ax_data.set_xlabel('Time (s)')
        ax_data.set_ylabel('Position (m)')
        ax_data.set_title('Position vs Time')
        ax_data.grid(True, alpha=0.3)
        
        line_true, = ax_data.plot([], [], 'b-', label='True Position', linewidth=2)
        line_measured, = ax_data.plot([], [], 'r.', label='Measured Position', alpha=0.6)
        line_estimated, = ax_data.plot([], [], 'g.', label='Estimated Position', alpha=0.6)
        ax_data.legend()
        
        plt.tight_layout()
        ani = None
        # update animation
        def animate(frame):
            self.time_index += 1
            t = frame * self.dt

            # update the state variables of the car
            acceleration = self.car.control(t)
            self.car.update(acceleration, self.dt)
            self.estimator.update(control=acceleration, x_t=self.estimator.x_estimate, type='Kalman Filter')
            
            # update the state variables of the observer
            measured_pos = self.observer.measure(self.car)

            # record data
            times.append(t)
            true_positions.append(self.car.position)
            measured_positions.append(measured_pos)
            estimated_positions.append(self.estimator.x_estimate[0,0])

            # update graphic elements
            car_patch.set_x(self.car.position)
            line_true.set_data(times, true_positions)
            line_measured.set_data(times, measured_positions)
            line_estimated.set_data(times, estimated_positions)
            if frame == self.time_steps - 1:
                if ani is not None:
                    ani.event_source.stop()
                fig.canvas.new_timer(interval=1, callbacks=[(plt.close, [fig], {})]).start()
            return car_patch, line_true, line_measured, line_estimated

        ani = FuncAnimation(fig, animate, 
                        frames=self.time_steps,
                        interval=int(1000 * self.dt),  # 100ms per frame
                        blit=True,
                        repeat=False)
            
        plt.show()
        return ani
    
if __name__ == "__main__":
    sim = Simulator()
    ani = sim.visualize()
    print("Simulation complete.")