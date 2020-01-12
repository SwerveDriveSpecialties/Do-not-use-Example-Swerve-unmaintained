# Example Swerve Project

When using Swerve Drive Specialties MK2 modules this template code will provide a quick and simple way to get your robot driving.

## Electrical Hardware Setup

1.	A navX should be plugged into the roboRIO MXP port.
2.	Steering encoders (analog US digital MA3) are connected to the roboRIO analog input ports.	
3.	Spark Max motor controllers for the drive motors are:
    1.	powered using 40 Amp PDP ports and breakers
    2.	controlled with CAN Bus
    3.  set to brushless brake mode (blinking cyan LED status when disabled) 
4.	Spark Max motor controllers for the steering motors are:
    1.	powered with either size PDP port. We recommend connecting them to the small ports with 30 Amp breakers
    2.	controlled with CAN Bus
    3.  set to brushless brake mode (blinking cyan LED status when disabled) 
    
The following port mapping is recommended

1.	Front Left Module
    1.	Drive Motor Controller – CAN ID 1
    2.	Steering Motor Controller – CAN ID 2
    3.	Steering Encoder – Analog input 0
2.	Front Right Module
    1.	Drive Motor Controller – CAN ID 3
    2.	Steering Motor Controller – CAN ID 4
    3.	Steering Encoder – Analog input 1
3.	Back Left Module
    1.	Drive Motor Controller – CAN ID 5
    2.	Steering Motor Controller – CAN ID 6
    3.	Steering Encoder – Analog input 2
4.	Back Right Module
    1.	Drive Motor Controller – CAN ID 7
    2.	Steering Motor Controller – CAN ID 8
    3.	Steering Encoder – Analog input 3

## Default Control Setup

By default the robot is setup to be controlled by a XBox One controller. However any XBox-style controller should work.

The left stick is setup to control the translational movement of the robot using field-oriented control.

The right stick is setup to control the rotational movement of the robot. Right on the stick should make the robot
rotate clockwise while left should make the robot rotate counter-clockwise.

The back button on the controller is setup to re-zero the robot's gyroscope. By default, the direction the robot is
facing when turned on is the forwards direction but this can be changed by re-zeroing the gyroscope.

## Configure For Your Robot

1. Set your team number using the WPILib extension's "Set Team Number" action.
2. In the `RobotMap` class:
    1. If needed, change the values to match the ports and CAN IDs on your robot.
3. In the `DrivetrainSubsystem` class:
    1. Set the `TRACKWIDTH` and `WHEELBASE` to your robot's trackwidth and wheelbase.
    2. Set all of the `*_ANGLE_OFFSET` constants to `-Math.toRadians(0.0)`.
4. Deploy the code to your robot.
    > NOTE: The robot isn't drivable quite yet, we still have to setup the module offsets
5. Turn the robot on its side and align all the wheels so they are facing in the forwards direction.
    > NOTE: The wheels will be pointed forwards (not backwards) when modules are turned so the large bevel gears are towards the right side of the robot. When aligning the wheels they must be as straight as possible. It is recommended to use a long strait edge such as a piece of 2x1 in order to make the wheels straight.
6. Record the angles of each module using the angle put onto Shuffleboard. The values are named
    `Front Left Module Angle`, `Front Right Module Angle`, etc.
7. Set the values of the `*_ANGLE_OFFSET` to `-Math.toRadians(<the angle you recorded>)`
    > NOTE: All angles must be in degrees.
8. Re-deploy and try to drive the robot forwards. All the wheels should stay parallel to each other. If not go back to
    step 3.
9. Make sure all the wheels are spinning in the correct direction. If not, add 180 degrees to the offset of each wheel
    that is spinning in the incorrect direction. i.e `-Math.toRadians(<angle> + 180.0)`.

### Optional Steps
#### Changing Controller Setup
To invert the controller sticks or modify the control mapping modify the `DriveCommand` class.

#### Using Different Types of Motors
While the default hardware setup uses NEOs & Spark MAXs to control the module, teams may desire to use different motors
to control their modules. The new `Mk2SwerveModuleBuilder` class supports any combination of NEOs, CIMs, or Mini CIMs
using either CAN or PWM.

##### Example 1
Angle motor: NEO controlled by a Spark MAX over CAN
Drive motor: NEO controlled by a Spark MAX over CAN

```java
SwerveModule module = new Mk2SwerveModuleBuilder(new Vector2(5.0, 5.0))
    .angleEncoder(new AnalogInput(0), -Math.toRadians(254.16))
    .angleMotor(new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless),
            Mk2SwerveModuleBuilder.MotorType.NEO)
    .driveMotor(new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless),
            Mk2SwerveModuleBuilder.MotorType.NEO)
    .build();
```

##### Example 2
Angle motor: NEO controlled by a Spark MAX over PWM with custom PID constants
Drive motor: NEO controlled by a Spark MAX over CAN


```java
SwerveModule module = new Mk2SwerveModuleBuilder(new Vector2(5.0, 5.0))
    .angleEncoder(new AnalogInput(0), -Math.toRadians(330.148))
    .angleMotor(new Spark(4), new PidConstants(1.0, 0.0, 0.001))
    .driveMotor(new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless),
            Mk2SwerveModuleBuilder.MotorType.NEO)
    .build();
```

##### Example 3
Angle motor: Mini CIM controlled by a Talon SRX over CAN
Drive motor: CIM controlled by a Talon SRX over CAN

```java
SwerveModule module = new Mk2SwerveModuleBuilder(new Vector2(5.0, 5.0))
    .angleEncoder(new AnalogInput(0), -Math.toRadians(118.1114))
    .angleMotor(new TalonSRX(4), Mk2SwerveModuleBuilder.MotorType.MINI_CIM)
    .driveMotor(new TalonSRX(5), Mk2SwerveModuleBuilder.MotorType.CIM)
    .build();
```