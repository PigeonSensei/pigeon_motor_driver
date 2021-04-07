# l298n

L298N Motor Driver Package

This Package is for RaspberryPi

### Dependency package
- wiringPi

- motor_driver_msgs

### Run

```bash
roslaunch l298n l298n.launch
```

### Subscribed Topics

- /motor_command (motor_driver_msgs/MotorCommand)

### Services
- SetMotorDriver (l298n/SetMotorDriver)

  -  command

     EMStop : E-stop mode on motor driver
     
     EMStopRelease : E-stop mode release on motor driver

### Parameters

~ IN1 (int, default: 21)

  Gpio Pin number corresponding to IN1 of motor driver
  
~ IN2 (int, default: 20)

  Gpio Pin number corresponding to IN2 of motor driver  

~ IN3 (int, default: 23)

  Gpio Pin number corresponding to IN3 of motor driver

~ IN4 (int, default: 24)

  Gpio Pin number corresponding to IN4 of motor driver

~ ENA (int, default: 12)

  Gpio Pin number corresponding to ENA of motor driver

~ ENB (int, default: 13)

  Gpio Pin number corresponding to ENB of motor driver
  
~ minimum_motor_command (int, default: 20)

  minimum value of motor_command
  
~ maximum_motor_command (int, default: 20)

  minimum value of motor_command

