Black Ice is a **Reactive Path Follower** developed by __FTC Team #18535__, Frozen Code, to provide more simple, efficient, effective path following by predicting real-time braking displacement. Unlike tradition path following libraries that gradually slowing down, Black Ice dynamically calculates the braking distance based on the robot's current speed. This enables the robot to predict its position, allowing the robot maintain full power for as long as possible, only beginning to braking at the optimal point. Also allows the robot to stay on curves.

By incorporating predicting the robot's position, the system provides:

Zero Arbitrary, Manual Tuning
Modular Customization
Smoother Transitions: Ensuring less jerky motions for precision-intensive tasks.
Faster Execution: Reducing time spent in path planning and execution.
Dynamic Adjustments: Reacting to obstacles or changes in the environment in real-time.

### Requirements: Pinpoint Odometry Processor

By having the robot follow points, this forces the robot to follow the path. Using dervivates like pedro paths adds complexity and adds transitional correction.

Support for Bezier Curves

Black Ice is tailored for teams looking for simple path execution with the option of modular customization.

Why did we develop Black Ice?
- To intuitively learn, hands-on, how path following works in robotics. We encourage you to learn by making your own drive and heading correction functions.
- Potentially find more efficient and simpler solutions
- Bring more modular customization
- To limit the tuning process
- Give people more options

# Usage

# Credits
All code developed by Jacob Ophoven with help of Coach Andy and members of the FTC community.


# How Black Ice works
Black Ice is a efficient and effective dynamic path follower developed by Team #18535, Frozen Code, in the 2025 season and offseason.

How is ours is different from others like Roadrunner and Pedro Path? The key difference is that Black Ice calculates braking distance by having the motors on zero power brake mode. Other libraries, calculate drift distance by having the motors on zero power float mode. Our framework uses the braking distance to predict where the robot will be next, allowing it to maintain full speed until it reaches an optimal braking point, minimizing unnecessary slowing while ensuring precise stopping.

We have just one tuning test that runs the robot at different velocities to brake and calculate the braking distance. With those data points, we used quadratic regression (since stopping distance is proportional to velocity squared) to derive a function that accurately predicts the required braking distance at any speed.

Our drive power formula is purely proportional, subtracting the predicted braking distance from the target position. No arbitrary constants are needed. No integral term is needed, as stationary robots have zero braking distance. No derivative term is required either, since braking distance naturally adjusts to slow the robot precisely when needed.

Pedro Pathing is an advanced Reactive Vector Follower developed by FTC Team 10158 to revolutionize autonomous navigation in robotics. Unlike conventional pathing systems such as RoadRunner, Pedro Pathing leverages Bézier curve generation to produce smoother, faster, and more efficient trajectories. Its primary focus is on enhancing the adaptability of robots during autonomous operation by reacting dynamically to environmental changes, reducing error margins, and ensuring optimal path execution.

By incorporating Bézier curves, the system provides:

Smoother Transitions: Ensuring less jerky motions for precision-intensive tasks. Faster Execution: Reducing time spent in path planning and execution. Dynamic Adjustments: Reacting to obstacles or changes in the environment in real-time. Pedro Pathing is tailored for teams looking to push the boundaries of autonomous efficiency and accuracy. Whether you’re a seasoned team or just getting started with autonomous systems, this documentation will guide you through setting up, tuning, and implementing Pedro Pathing in your projects.
