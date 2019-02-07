//TO DO: better documentation and commenting lol xd kill me

package org.firstinspires.ftc.teamcode;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "kyle copy pasted", group = "Pushbot")

public class KyleAndEduardoGoSuperSaiyan extends LinearOpMode
{

    private final double CLAW_SPEED = 0.02;                     // sets rate to move servo
    private final double CLAW_OPEN_MAX = -0.12;
    private final double CLAW_OPEN_MIN = -0.5;
    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();    // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private double clawOffset = 0;                        // Servo mid position

    @Override
    public void runOpMode()
    {
        double left;
        double right;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "I AM A BAD ROBOT PUNISH ME FATHER");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Combine drive and turn for blended motion.
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            // Normalize the values so neither exceed +/- 1.0
            left = Range.clip(left, -1, 1);
            right = Range.clip(right, -1, 1);

            // Output the safe vales to the motor drives.
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.b)
            {
                clawOffset += CLAW_SPEED;
            }

            else if (gamepad1.x)
            {
                clawOffset -= CLAW_SPEED;
            }

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(HardwarePushbot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(HardwarePushbot.MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad1.y)
            {
                robot.arm.setPower(HardwarePushbot.ARM_UP_POWER);
            }
            else if (gamepad1.a)
            {
                robot.arm.setPower(HardwarePushbot.ARM_DOWN_POWER);
            }
            else
            {
                robot.arm.setPower(0.0);
            }

            if (clawOffset > CLAW_OPEN_MAX)
            {
                clawOffset = CLAW_OPEN_MAX;
            }
            if (clawOffset < CLAW_OPEN_MIN)
            {
                clawOffset = CLAW_OPEN_MIN;
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("claw", "Offset = %.2f", clawOffset);
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(25);
        }
    }
}