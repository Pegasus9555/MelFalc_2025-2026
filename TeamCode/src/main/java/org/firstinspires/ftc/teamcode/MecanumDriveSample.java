package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.configuration.Utility;
//import org.firstinspires.ftc.teamcode;


@TeleOp(name="Teleop Test Mecanum Drive", group="19380")
@Disabled
public class MecanumDriveSample extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    double driveMultiplicative = 0.8;
    boolean turboMode = false;
    double rotationMultiplicative = 0.75;


    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "fldrive");
        front_right  = hardwareMap.get(DcMotor.class, "frdrive");
        back_left    = hardwareMap.get(DcMotor.class, "bldrive");
        back_right   = hardwareMap.get(DcMotor.class, "brdrive");
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        //double drive = Utility.deadStick(-gamepad1.left_stick_y);
        //double strafe = Utility.deadStick(gamepad1.left_stick_x);
        //double twist  = Utility.deadStick(gamepad1.right_stick_x);
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;

        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.

        double leftFrontPower  = drive + strafe + (rotationMultiplicative*twist);
        double rightFrontPower = drive - strafe - (rotationMultiplicative*twist);
        double leftBackPower   = drive - strafe + (rotationMultiplicative*twist);
        double rightBackPower  = drive + strafe - (rotationMultiplicative*twist);
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.

        // apply the calculated values to the motors.

 /*       if(gamepad1.right_bumper){
            if(turboMode = false){
                turboMode = true;
                driveMultiplicative = 1;
            }
            if(turboMode = true){
                turboMode = false;
                driveMultiplicative = 0.7;
            }
        }*/

        front_left.setPower(driveMultiplicative*leftFrontPower);
        front_right.setPower(driveMultiplicative*rightFrontPower);
        back_left.setPower(driveMultiplicative*leftBackPower);
        back_right.setPower(driveMultiplicative*rightBackPower);



      //  telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        //telemetry.update();


    }
}