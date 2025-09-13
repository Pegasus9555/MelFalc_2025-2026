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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
 */


@TeleOp(name="MelFalc2024-DEV-TelOp-SlidingArm-TEST", group="Robot")
@Disabled
public class MelFalc_2024_IntoTheDeep_TESTLIFTER_TEMP extends LinearOpMode {

    /* Declare OpMode members. */

    //Declare motors for slider system
   /* private DcMotor slider_left = null;*/
    private DcMotor slider_right = null;


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation




    @Override
    public void runOpMode() {

        /* Define and Initialize Motors */


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        /*slider_left = hardwareMap.get(DcMotor.class, "sliderleft");
        slider_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider_left.setDirection(DcMotor.Direction.FORWARD);*/
        slider_right = hardwareMap.get(DcMotor.class, "sliderright");
        slider_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider_right.setDirection(DcMotor.Direction.FORWARD);


        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        //((DcMotorEx) slider_left).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) slider_right).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                //slider_left.setPower(1);
                slider_right.setPower(1);
            } else if (gamepad1.left_bumper) {
                //slider_left.setPower(-1);
                slider_right.setPower(-1);
            }
            else {
                //slider_left.setPower(0);
                slider_right.setPower(0);
            }

        }
    }
}

