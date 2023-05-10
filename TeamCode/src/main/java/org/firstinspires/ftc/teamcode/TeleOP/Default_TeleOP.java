// This file needs to be replaced with the BlocksToJava export.
// This opmode has been broken for a while.

package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp
public class Default_TeleOP extends LinearOpMode {

  private DcMotor FrontLeft;
  private DcMotor BackLeft;
  private DcMotor FrontRight;
  private DcMotor BackRight;
  private DcMotor Elevator_Motor;
  private Servo LClaw;
  private Servo RClaw;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double y;
    double x;
    double rx;
    double denominator;
    float tgtpower;

    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");
    Elevator_Motor = hardwareMap.get(DcMotor.class, "Elevator_Motor");
    LClaw = hardwareMap.get(Servo.class, "LClaw");
    RClaw = hardwareMap.get(Servo.class, "RClaw");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        
        y = gamepad1.left_stick_y * 0.5;
        x = -gamepad1.left_stick_x * 0.5;
        rx = -gamepad1.right_stick_x * 0.6;
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
        
        // Chassis Control
        FrontLeft.setPower(-(y + rx + x) / denominator);
        BackLeft.setPower(-((y + rx) - x) / denominator);
        FrontRight.setPower(-((y - x) - rx) / denominator);
        BackRight.setPower(-((y - rx) + x) / denominator);
        
        // Slider Control 
        tgtpower = gamepad2.left_stick_y;
        Elevator_Motor.setPower(-tgtpower);
        
        // Claw Control 
        if (gamepad2.a) {
          LClaw.setPosition(0.4);
          RClaw.setPosition(0.95);
        }
        if (gamepad2.b) {
          LClaw.setPosition(0.65);
          RClaw.setPosition(0.75);
        }
        telemetry.update();
      }
    }
  }
}
