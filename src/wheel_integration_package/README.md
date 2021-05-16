# Wheel Integration Package
- This package integrates the Pro Controller with the wheel motors
-  This package contains a single node called integration_node.

# Nodes
## integration_node
- Subscribes to the topic **"cmd_vel"** from where it reads Geometry Twist message from the Pro Controller
- Publishes to the following topics:
	- **"/integration_node/lwheels_pub_topic"** - The velocity of the left wheels is published to this topic as a Geomerty Twist Message
	- **"/integration_node/rwheels_pub_topic"** - The velocity of the right wheels is published to this topic as a Geomerty Twist Message

![integration_node](https://user-images.githubusercontent.com/56468618/113475473-91135a00-9493-11eb-923b-db406d3fbfd6.jpg)

# Velocity Calculation
![wheel_calc](https://user-images.githubusercontent.com/56468618/113476027-4e06b600-9496-11eb-9f92-440c9622fcde.jpg)

We must find V<sub>l</sub> and V<sub>r</sub> that correspond to the velocities of the left and right wheels of the rover, from &omega;.
From circular motion, we know the follwing about speed <img src="https://render.githubusercontent.com/render/math?math=v">:

![equation](https://latex.codecogs.com/gif.latex?\dpi{150}&space;v&space;=&space;\frac{2\pi&space;R}{T})

We also know that angular velocity <img src="https://render.githubusercontent.com/render/math?math=\omega  = \frac{2\pi}{T}">, which makes

![equation](https://latex.codecogs.com/gif.latex?\dpi{150}&space;v=\omega&space;R)

So the Radius of Convergence <img src="https://render.githubusercontent.com/render/math?math=R"> can be found by 

![equation](https://latex.codecogs.com/gif.latex?\dpi{200}&space;R&space;=&space;\frac{v}{\omega})

Since <img src="https://render.githubusercontent.com/render/math?math=v"> originates from the center of the rover, we know that 

![equation](https://latex.codecogs.com/gif.latex?\dpi{200}&space;V_l&space;=&space;\omega&space;\Big(R-&space;\frac{\text{wheel&space;dist}}{2}\Big))

![equation](https://latex.codecogs.com/gif.latex?\dpi{200}&space;V_r&space;=&space;\omega&space;\Big(R&plus;&space;\frac{\text{wheel&space;dist}}{2}\Big))

The above two equations calculate the speed of the left and right wheels. Since the distance of left wheel from the Instantaneous Center of Curvature <img src="https://render.githubusercontent.com/render/math?math=ICC"> is <img src="https://render.githubusercontent.com/render/math?math=\Big(R-\frac{\text{wheel dist}}{2}\Big)">, the speed of the left wheel is <img src="https://render.githubusercontent.com/render/math?math=V_l = \omega \Big(R- \frac{\text{wheel dist}}{2}\Big))">. 

Similarly, the distance of right wheel from the Instantaneous Center of Curvature <img src="https://render.githubusercontent.com/render/math?math=ICC"> is ![equation](https://latex.codecogs.com/gif.latex?\dpi{80}&space;\Big(R&space;&plus;&space;\frac{\text{wheel&space;dist}}{2}\Big)); therefore its speed is ![equation](https://latex.codecogs.com/gif.latex?\dpi{80}&space;V_r&space;=&space;\omega&space;\Big(R&plus;&space;\frac{\text{wheel&space;dist}}{2}\Big)).

These equations are used in the calculations done in the ``subscriberCallBack()`` function. 
