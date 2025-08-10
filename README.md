## Project Overview and Goals
Over the summer I created a robot that balances using only two wheels.The robot utilizes a PID control loop and data fed from an accelerometer to maintain an upright orientation. I primarily started this project to learn more about STM32 microcontrollers and how to use their onboard peripherals; however, I also wanted to apply the theory I learned from my linear control systems class.

https://github.com/user-attachments/assets/53945460-7c7f-4bdc-ada0-24642ccd9668

![Two Wheeled Robot](assets/images/IMG_3527.JPG)

## Design and Component Selection
The main design constraint was cost since the main purpose of the project was just to learn. What I did not anticipate from setting this design constraint was that it also made the implementation of the control system to be more challenging, which in turn helped me learn more.

The design had to be capable of accomplishing multiple tasks. Below are some of the tasks and how the design incorporated them.
 
### Computation
For the microcontroller, I chose the STM32F303K8 on the NUCLEO board. I wanted to use an STM32 microcontroller since the knowledge that it takes to use them can easily be transferred to multiple other vendors that are also based off of the ARM cortex M microprocessor architecture. 

### Data Collection
The MPU6050 is a MEMs sensor that essentially functions as both an accelerometer and gyroscope. I used the STM32’s HAL libraries to establish communication between the microcontroller and sensor. The sensor’s interrupt pin was attached to a pin configured in external interrupt mode on the microcontroller. To get the accelerometer and gyro data, I read registers on the MPU6050 using the I2C protocol every time the external interrupt fired. 

To get the orientation (yaw, pitch, roll), sensor fusion had to be done. While there are many ways to do sensor fusion, I chose to use a Kalman filter. I chose to not use the digital motion processor built into the MPU6050 because I wanted to learn about digital kalman filters, which was left out of my linear control systems course. In hindsight, it might have been better to use an extended kalman filter since the system was not linear, but it really did not seem to cause any issues. 

### Actuators
For moving the wheels, I chose to use the typical yellow TT motors. They have a x:1 gear reduction ratio, so they generate enough torque to allow movement. I originally did try using motors I had on hand, but they just could not provide enough torque.

To drive the motors in both directions, I chose an LM298 breakoutboard. The LM298 features an H-bridge which allows the motors to be spun in both directions. PWM signals from the STM32 board, which the duty cycle is determined from the PID controller’s output, are sent to the breakout board to control the power output of the motors.

### Control
My linear control systems class mainly taught on analog systems. However, since I wanted to implement the PID controller on a microcontroller, I had to convert my analog knowledge to digital. 

I used the trilinear transformation, and then I took the Z transform inverse to turn the analog PID controller into a digital one. From my research, the trilinear transform usually does a decent job of allowing the digital system to match the analog in behavior.

The PID control was the most challenging part of the project, and there were many flaws in the initial PID controller’s design. Initially, for the integral term, I chose a clamp integrator. This was one of the flaws in my original design, but originally I thought it was good to prevent windup (the output of the integral term could go to infinity without proper measures). Since I clamped the integrator so that only enough was outputted to get 100% duty cycle, the integral term would frequently get set to 0 and would not contribute at all in the control effort. As a result, the robot would have a tendency to tilt to one side and eventually fall over.  Once I completely got rid of any clamping, the robot started balancing much better since the integral term would eliminate the tendencies. Clamping of the integral term is not really necessary since the robot is constantly tilting back and forth, not allowing the integral term to go to infinity. Another issue I had was that I could not get the motor to move in small increments because there was no direct position control. It turns out that I was not using the derivative term properly. The derivative term allows the motor to make small steps. I was using a pseudodifferentiator, which basically is the pure derivative and a lowpass filter right after. The issue was that the cutoff frequency I chose was too low, which made the derivative term to have too much delay for the robot to properly balance.

The control loop runs at 250 Hz. 

## Results and Takeaways
To make a self balancing robotThe I2C bus speed was 400 kHz
