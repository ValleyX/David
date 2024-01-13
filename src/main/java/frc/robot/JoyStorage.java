package frc.robot;

import java.io.Serializable;

public class JoyStorage implements Serializable {

        public double leftdriveYstick; //swerve forwards and backwards
        public double leftdriveXStick; //swerve left and right
        public double rightdriveXstick; //swerve rotation
        public double manipulatorLeftYstick;

        public boolean buttonA; // lift to low
        public boolean buttonB; // lift to mid
        public boolean buttonY; // lift to high
        public boolean buttonX; //lift  reset ;
        public boolean buttonR3;
        public boolean buttonL3;
        public boolean buttonBack;

        public boolean bumperLeftExpell;
        public boolean bumperRightIntake;

        public boolean driverButtonR3;
        public boolean driverButtonL3;
        



    public JoyStorage() {
    }

    public JoyStorage(
        double leftdriveYstick,
        double leftdriveXStick,
        double rightdriveXstick,
        double manipulatorLeftYstick
        ) {
        this.leftdriveYstick = leftdriveYstick;
        this.leftdriveXStick = leftdriveXStick;
        this.rightdriveXstick = rightdriveXstick;
        this.manipulatorLeftYstick = manipulatorLeftYstick;
      
    }

    // getters and setters, toString() .... (omitted for brevity)
}