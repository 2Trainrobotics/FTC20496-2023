package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotBase
{
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor elevator = null;
    // private DcMotor turretRotation = null;
    public DcMotor launcher = null;
    public Servo gripServo = null;

    // Elevator variables and constants
    // Max vertical displacement from zero:  ~150-160
    public static final int ELEVATOR_MAX_DISPLACEMENT = 155;
    public static final double ELEVATOR_POWER_MAINTAIN_POSITION = 0.4;
 
    // Grip Servo
    public static final double GRIP_OPEN_POSITION = 0.25;
    public static final double GRIP_CLOSED_POSITION = 0.03;

    HardwareMap hardwareMap = null;

    /* Constructor */
    public RobotBase() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        gripServo = hardwareMap.get(Servo.class, "gripServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        
        // Set up elevator motors
        elevator.setDirection(DcMotor.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Launcher
        launcher.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        elevator.setPower(0);
    }
}
