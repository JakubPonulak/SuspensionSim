# SuspensionSim
SuspensionSim is a MATLAB app which simulates a car suspension system and how it would react to small bumps on the road.


The purpose of this application is to provide a simplified simulation of a typical car suspension system, one which is travelling at approximately 10 m/s (22.4 mph) and has been subjected to a small bump while travelling on a flat road. Users can enter their desired data values, click the "Run Simulation" button, and watch the simulation play in the display section. After the simulation is complete, the application will also display a plot of Height vs Time, on which both the car body and wheel will be graphed independantly.

![alt text](https://github.com/JakubPonulak/SuspensionSim/blob/main/sample_image.png)

## User Input
The application allows users to choose 3 different values that affect the results of the simulation: 
- Height of the bump in meters
- Radius of the wheel in meters
- Stiffness of the tire in kN/m

Due to contraints with the model and the ode78 function, the maximum allowable bump height is 0.6 m, while the tire stiffness must be set between 100 and 200 kN/m. While the wheel radius does not affect the results of the data, it has been restricted between values of 0.25 and 0.6 for realism and to ensure that all wheel sizes will fit within the animation region.

## Equations of Motion for the Suspension System

The system consists of:
- **Sprung mass** ($M_s$): Represents the car body.
- **Unsprung mass** ($M_u$): Represents the wheel.
- **Road profile** ($y_r$): Defines the bump shape.

#### **State Variables**
- $y_c$ - Displacement of the car body  
- $\dot{y_c}$ - Velocity of the car body  
- $y_w$ - Displacement of the wheel  
- $\dot{y_w}$ - Velocity of the wheel  

### **Equations of Motion**
#### **For the Sprung Mass (Car Body)**

$M_s \ddot{y_c} = - c_s (\dot{y_c} - \dot{y_w}) - k_s (y_c - y_w)$

Solving for $\ddot{y_c}$:

$$\ddot{y_c} = \frac{- c_s (\dot{y_c} - \dot{y_w}) - k_s (y_c - y_w)}{M_s}$$

#### **For the Unsprung Mass (Wheel)**

$M_u \ddot{y_w} = c_s (\dot{y_c} - \dot{y_w}) + k_s (y_c - y_w) - k_t (y_w - y_r)$

Solving for $\ddot{y_w}$:

$$\ddot{y_w} = \frac{c_s (\dot{y_c} - \dot{y_w}) + k_s (y_c - y_w) - k_t (y_w - y_r)}{M_u}$$

### **Bump Profile (Road Input)**
The bump is modeled as:

$$y_r =
\begin{cases} 
h \sin^2 \left( \frac{\pi t}{T} \right), & \text{if } t \leq T \\
0, & \text{otherwise}
\end{cases}$$

Where:
- $k_s$ = Suspension stiffness (N/m)  
- $k_t$ = Tire stiffness (N/m)  
- $c_s$ = Damping coefficient (Ns/m)  
- $h$ = Bump height  
- $T$ = Bump duration


