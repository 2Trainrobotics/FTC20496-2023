
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;
  
  
  /*
   * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
   * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
   * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
   * class is instantiated on the Robot Controller and executed.
   *
   * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
   * It includes all the skeletal structure that all linear OpModes contain.
   *
   * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
   * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
   */
  
@TeleOp(name="2C Tank Drive", group="Linear OpMode")
public class TeleopTankDrive2C extends LinearOpMode 
{
    // Declare OpMode members.
    private RobotBase robot = new RobotBase();
    private ElapsedTime runtime = new ElapsedTime();
    //  private DcMotor leftDrive = null;
    //  private DcMotor rightDrive = null;
    //  private DcMotor elevator = null;
    //  // private DcMotor turretRotation = null;
    //  private DcMotor launcher = null;
    //  private Servo gripServo = null;


    // Elevator variables and constants
    // Max vertical displacement from zero:  ~150-160
    private int elevatorStartingPosition;
    private int elevatorMaxPosition;
    // private static final int ELEVATOR_MAX_DISPLACEMENT = 155;
    // private static final double ELEVATOR_POWER_MAINTAIN_POSITION = 0.4;
 
    // Grip Servo
    // private static final double GRIP_OPEN_POSITION = 0.25;
    // private static final double GRIP_CLOSED_POSITION = 0.03;
  
 
    // Turret Rotation variables and constants
    // left:  -70 to 65
    // right:  -93 to -445
    // private int turretRotationStartingPosition;
    // private int turretRotationMaxLeftPosition;
    // private int turretRotationMaxRightPosition;
    // private static final int TURRET_ROTATION_MAX_LEFT_DISPLACEMENT = 135;
    // private static final int TURRET_ROTATION_MAX_RIGHT_DISPLACEMENT = -350;
 
 
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
  
        // CAU TODO:  move motor initialization to separate class so it can be reused with autononous
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        // rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        // elevator = hardwareMap.get(DcMotor.class, "elevator");
        // // turretRotation = hardwareMap.get(DcMotor.class, "turretRotation");
        // launcher = hardwareMap.get(DcMotor.class, "launcher");
        // gripServo = hardwareMap.get(Servo.class, "gripServo");
 
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // leftDrive.setDirection(DcMotor.Direction.FORWARD);
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);
 
        // Elevator direction set so that positive power moves the arm up.
        // elevator.setDirection(DcMotor.Direction.REVERSE);
        // elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorStartingPosition = robot.elevator.getCurrentPosition();
        elevatorMaxPosition = elevatorStartingPosition + robot.ELEVATOR_MAX_DISPLACEMENT;
 
        // Turret rotation direction set so that positive power moves the tower right.
        // turretRotation.setDirection(DcMotor.Direction.FORWARD);
        // turretRotationStartingPosition = turretRotation.getCurrentPosition();
        // turretRotationMaxLeftPosition = turretRotationStartingPosition + TURRET_ROTATION_MAX_LEFT_DISPLACEMENT;
        // turretRotationMaxRightPosition = turretRotationStartingPosition + TURRET_ROTATION_MAX_RIGHT_DISPLACEMENT;
 
        // Launcher
        // launcher.setDirection(DcMotor.Direction.REVERSE);
 
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
  
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
  
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
  
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
  
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            // double drive = -gamepad1.left_stick_y;
            // double turn  =  gamepad1.right_stick_x;
            // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            // rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
  
            // Send calculated power to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
 
            // Elevator up/down
            int elevatorCurrentPosition = robot.elevator.getCurrentPosition();
            double elevatorPower = 0;
            // elevatorPower = robot.ELEVATOR_POWER_MAINTAIN_POSITION;
            if (gamepad2.left_stick_y < -0.5) {
                if (elevatorCurrentPosition < elevatorMaxPosition)
                    elevatorPower = 0.6;
            } else if (gamepad2.left_stick_y > 0.5) {
                if (elevatorCurrentPosition > elevatorStartingPosition)
                    elevatorPower = 0;
            }
            robot.elevator.setPower(elevatorPower);   

            // Turret Rotation
            // int turretRotationCurrentPosition = turretRotation.getCurrentPosition();
            // double turretRotationPower = 0;
            // if (gamepad1.x) {
            //   if (turretRotationCurrentPosition < turretRotationMaxLeftPosition)
            //         turretRotationPower = 0.4;
            // } else if (gamepad1.b) {
            //   if (turretRotationCurrentPosition > turretRotationMaxRightPosition)
            //         turretRotationPower = -0.4;
            // }
            // turretRotation.setPower(turretRotationPower);
 
            // Gripper
            if (gamepad2.right_stick_y > 0.5) {
                robot.gripServo.setPosition(robot.GRIP_CLOSED_POSITION);
            } else if (gamepad2.right_stick_y < -0.5) {
                robot.gripServo.setPosition(robot.GRIP_OPEN_POSITION);
            }
 
            // Paper airplane launcher
            double launcherPower = 0;
            boolean launchAirplane = (gamepad1.left_trigger > 0.5) && (gamepad1.right_trigger > 0.5);
            if (launchAirplane) {
                launcherPower = 1.0;
            }
            robot.launcher.setPower(launcherPower);
 
            try {
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                // telemetry.addData("Elevator", "power (%.3f), position (%d)", elevatorPower, elevatorCurrentPosition);
                // telemetry.addData("Elevator", "start (%d), max (%d)", elevatorStartingPosition, elevatorMaxPosition);
                // telemetry.addData("Rotation", "power (%.3f), position (%d)", turretRotationPower, turretRotationCurrentPosition);
                telemetry.update();
            }
            catch (Exception e) {
            }
        }
    }
 }
 
