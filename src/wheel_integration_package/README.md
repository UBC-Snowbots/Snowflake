# Wheel Integration Package
- This package is integrated the Pro Controller with the wheel motors
-  This package contains a single note called integration_node.

# Nodes
## integration_node
- Subscribes to the topic **"cmd_vel"** from where it reads Geometry Twist message from the Pro Controller
- Publishes to the following topics:
	- **"/integration_node/lwheels_pub_topic"** - The velocity of the left wheels is published to this topic as a Geomerty Twist Message
	- **"/integration_node/rwheels_pub_topic"** - The velocity of the right wheels is published to this topic as a Geomerty Twist Message

![integration_node](https://user-images.githubusercontent.com/56468618/113475473-91135a00-9493-11eb-923b-db406d3fbfd6.jpg)


# Velocity Calculation
![Illustration of Differential Drive](https://lh4.googleusercontent.com/TqfDBFzATE9TtIoKtKU0MUVGNphvDLySSMhwtvjxkst8QvyeYdZe19bDjR-qaVYC2cE42rRqZ-jlg7BV29CooRGWkpI6e7jqck8r2DoL59AfSa0Xr_KfzmqIKNWCurI2Ow=w1280)
We must find V<sub>l</sub> and V<sub>r</sub> that correspond to the velocities of the left and right wheels of the rover, from &omega;.
From circular motion, we know the follwing about speed $v$: $$v = \frac{2\pi R}{T}$$We also know that angular velocity $\omega  = \frac{2\pi}{T}$, which makes
$$v=\omega R$$ So the Radius of Convergence $R$ can be found by $$R = \frac{v}{\omega}$$Since $v$ originates from the center of the rover, we know that $$V_l = \omega \Big(R- \frac{\text{wheel dist}}{2}\Big)$$ $$V_r = \omega \Big(R+ \frac{\text{wheel dist}}{2}\Big)$$The above two equations calculate the speed of the left and right wheels. Since the distance of left wheel from the Instantaneous Center of Curvature $(ICC)$ is $\Big(R-\frac{\text{wheel dist}}{2}\Big)$, the speed of the left wheel is $V_l = \omega \Big(R- \frac{\text{wheel dist}}{2}\Big)$. 
Similarly, the distance of right wheel from the Instantaneous Center of Curvature $(ICC)$ is $\Big(R+\frac{\text{wheel dist}}{2}\Big)$; therefore its speed is $V_r = \omega \Big(R+ \frac{\text{wheel dist}}{2}\Big)$.

These equations are used in the calculations done in the subscriberCallBack() function. 
