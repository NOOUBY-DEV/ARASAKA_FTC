package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "turret tuning", group = "testing")
public class turret_tuning extends LinearOpMode
{
        DcMotorEx turret_motor;

        @Override
        public void runOpMode()
        {
                final double MAX_ROTATE_SPEED = 0.4;

                turret_motor = hardwareMap.get(DcMotorEx.class, "turret_motor");

                turret_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                turret_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                waitForStart();

                while (opModeIsActive())
                {
                        final double rotate_power = gamepad1.left_stick_x * MAX_ROTATE_SPEED;

                        turret_motor.setPower(rotate_power);

                        telemetry.addData("ticks_traveled :- ", turret_motor.getCurrentPosition());

                        telemetry.update();
                }
        }
}
