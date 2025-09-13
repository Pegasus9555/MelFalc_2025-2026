/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]Th
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This is the Millennium Falcons 2024/2025 Robot Code
 * This code is based on the GoBilda starter kit and code, but with some key modifications
 *
 * This robot was originally designed with a two-motor differential-steered (sometimes called tank
 * or skid steer) drivetrain with a left and right drive motor.
 *
 * We are added Mechanum drives to this robot and are replacing the original Tank code
 * with Mechanum-compatable 4 motor controls which adds strafe capability
 *
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.
 *
 * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
 * external 5:1 reduction. This creates a total ~254.47:1 reduction.
 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make super sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * We have ensured this by using a physical stop in the stored arm position to set the same
 * startpoint each time.  This is a hardware solution to a software requirement
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 *
 * The below section is details on encoder ticks for arm angle.
 *
 * This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree.

    These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it.
    *
    * */


@Autonomous(name="MelFalc2024-AUTO_TEST", group="Robot")
//@Autonomous(name="MelFalc2024-PROD-Autonomous-FixedArm-PROD", group="Robot")
@Disabled
public class MelFalc_2024_IntoTheDeep_PLAYGROUND extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor armMotor = null; //the arm motor
    public CRServo intake = null; //the active intake servo
    public Servo wrist = null; //the wrist servo

    //Declare Mechanum Wheel Motors
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    double driveMultiplicative = 0.8;
    boolean turboMode = false;
    double rotationMultiplicative = 0.75;

    //Set constants for ARM Ticks.  See description above

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode() {

        /* Define and Initialize Motors */

        front_left = hardwareMap.get(DcMotor.class, "fldrive");
        front_right = hardwareMap.get(DcMotor.class, "frdrive");
        back_left = hardwareMap.get(DcMotor.class, "bldrive");
        back_right = hardwareMap.get(DcMotor.class, "brdrive");

        //Set motor directions.  Changing these will impact direction based on input
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */

        armMotor = hardwareMap.get(DcMotor.class, "armmotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        /* Set telemetry so that .update() does *not* clear the screen */
        telemetry.setAutoClear(false);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.addLine("Autonomous Mode!");
        telemetry.addData("Ticks per degree: ", ARM_TICKS_PER_DEGREE);
        telemetry.addLine(" ");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        // Move away from the wall
        telemetry.addLine("Hanging a specimen ...");
        telemetry.addLine(". . Strafing away from the wall.");
        telemetry.update();
        strafeRight(300);

        // Turn to position for hanging specimen
        telemetry.addLine(". . Rotating CW 90 degrees.");
        telemetry.update();
        rotateCW(750);

        // Set wrist for hanging specimen
        telemetry.addLine(". . Setting wrist to correct position.");
        telemetry.update();
        wrist.setPosition(0.80);

        sleep(5000);

        // Move arm into position to hang specimen
        armPosition = -160 * ARM_TICKS_PER_DEGREE;
        telemetry.addData(". . Moving arm to hanging armPosition = ", armPosition);
        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(2100);
        telemetry.update();

        // Move forward to hang specimen
        telemetry.addLine(". . Moving forward to hang.");
        telemetry.update();
        forward(400);

        // Moves for hanging specimen will go here go here
        telemetry.addLine("Waiting 5 seconds. Code to hang specimen will go here.");
        telemetry.update();
        sleep(5000);

        // Position arm for parking
        telemetry.update();
        armPosition = -120 * ARM_TICKS_PER_DEGREE;
        telemetry.addLine(" ");
        telemetry.addLine("Parking.");
        telemetry.addData(". . Moving arm to parking armPosition = ", armPosition);
        telemetry.update();
        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(2100);
        telemetry.update();

        telemetry.addLine(". . Backing up a bit and strafing left to move to parking.");
        telemetry.update();
        backward(400);
        strafeLeft(800);

        telemetry.addLine(". . Rotating CW to move to parking.");
        telemetry.update();
        rotateCW(775);

        telemetry.addLine(". . Strafing left to position for parking.");
        telemetry.update();
        strafeLeft(200);

        telemetry.addLine(". . Moving forward to position for parking.");
        telemetry.update();
        forward(200);

        telemetry.addLine("Waiting 5 seconds. Code to move to parking position will go here.");
        telemetry.update();
        sleep(5000);

        // Return arm to 0
        telemetry.addLine("Returning armPosition to 0.");
        telemetry.update();
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(2100);

        telemetry.addLine(" .. Sleeping 5 seconds.");
        telemetry.update();
        sleep(5000);

        if (((DcMotorEx) armMotor).isOverCurrent()) {
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }
        telemetry.addLine("Autonomous COMPLETE");
        telemetry.update();

        telemetry.addLine(" .. Sleeping 5 seconds.");
        telemetry.update();
        sleep(5000);

        /*
        telemetry.addLine("Driving in a square.");
        strafeLeft(1000);
        forward(1000);
        strafeRight(1000);
        backward(1000);

        telemetry.addLine("Moving wrist.");
        wrist.setPosition(0.47);

        int armCurrPosition = armMotor.getCurrentPosition();
        telemetry.addData("Current arm position: ", armCurrPosition);
//        telemetry.update();

        telemetry.addLine("Moving arm.");
        armPosition = -160 * ARM_TICKS_PER_DEGREE;
        telemetry.addData("ArmPosition: ", armPosition);
        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(2100);
        sleep(2000);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(2100);

        if (((DcMotorEx) armMotor).isOverCurrent()) {
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }
        telemetry.addLine("ArmPosition COMPLETE");
        telemetry.update();

        sleep(10000);

        //Drive Left? for 2.5 seconds at 60% speed
        drive(0, 0.6, 0);
        sleep(2500);
        drive(0, 0, 0);
        sleep(100);

        //Drive forward for 2.5 Seconds at 60% speed
        drive(0.6,0,0);
        sleep(2500);
        drive(0,0,0);
        sleep(100);

        //Turn right? for 0.5 seconds at 60% power
        drive(0,0,0.6);
        sleep(500);
        drive(0,0,0);

        //End Drive Mode

        telemetry.addLine("Drive Mode Complete");
        telemetry.update();
        sleep(500);
        telemetry.addData("ArmPosition: ", armPosition);
        telemetry.update();
        armMotor.setTargetPosition((int)ARM_SCORE_SAMPLE_IN_LOW);
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(2100);
        sleep(500);
        telemetry.addLine("ArmPosition COMPLETE");
        telemetry.update();
*/
        //wait(!opModeIsActive());

        /* Run until the driver presses stop */
        /*
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            drive(drive,strafe,twist);

            if (gamepad2.a) {
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad2.x) {
                intake.setPower(INTAKE_OFF);
            } else if (gamepad2.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }


            if (gamepad2.right_bumper) {
                armPosition = ARM_COLLECT;
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad2.left_bumper) {
                armPosition = ARM_CLEAR_BARRIER;
            } else if (gamepad2.y) {
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            } else if (gamepad2.dpad_left) {
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_right) {
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_up) {
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_down) {
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));


            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("Intake Power: ", intake.getPower());
            telemetry.update();

        }*/
    }

    public void drive (double drive, double strafe, double twist){

        double leftFrontPower = drive + strafe + (rotationMultiplicative * twist);
        double rightFrontPower = drive - strafe - (rotationMultiplicative * twist);
        double leftBackPower = drive - strafe + (rotationMultiplicative * twist);
        double rightBackPower = drive + strafe - (rotationMultiplicative * twist);
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        front_left.setPower(driveMultiplicative * leftFrontPower);
        front_right.setPower(driveMultiplicative * rightFrontPower);
        back_left.setPower(driveMultiplicative * leftBackPower);
        back_right.setPower(driveMultiplicative * rightBackPower);
    }
    private void strafeLeft(int duration) {
        drive(0, 0.6, 0);
        sleep(duration);
        drive(0, 0, 0);
        sleep(100);
    }

    private void strafeRight(int duration) {
        drive(0, -0.6, 0);
        sleep(duration);
        drive(0, 0, 0);
        sleep(100);
    }

    private void forward(int duration) {
        drive(-0.6, 0, 0);
        sleep(duration);
        drive(0, 0, 0);
        sleep(100);
    }

    private void backward(int duration) {
        drive(0.6, 0, 0);
        sleep(duration);
        drive(0, 0, 0);
        sleep(100);
    }

    private void rotateCW (int duration) {
        drive(0, 0, -0.75);
        sleep(duration);
        drive(0, 0, 0);
        sleep(100);
    }

    private void rotateCCW (int duration) {
        drive(0, 0, 0.75);
        sleep(duration);
        drive(0, 0, 0);
        sleep(100);
    }

}

