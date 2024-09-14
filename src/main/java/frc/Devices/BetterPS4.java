package frc.Devices;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.Container;
import frc.lib.util.Vector2;

public class BetterPS4 extends PS4Controller {
    Container<Boolean> povChanged = new Container<Boolean>(false);

    public BetterPS4(int port) {
        super(port);

        var con = this;

        Container<Integer> lastPOV = new Container<Integer>(-1);

        CommandScheduler.getInstance().schedule(new Command() {
            @Override
            public void execute() {
                povChanged.val = false;
                if (lastPOV.val != con.getPOV()) {
                    povChanged.val = true;
                    lastPOV.val = con.getPOV();
                }
                super.execute();
            }
        });
    }

    public boolean povChanged() {
        return povChanged.val;
    }

    public Vector2 getLeftStick() {
        return new Vector2(getLeftX(), -getLeftY());
    }

    public Vector2 getRightStick() {
        return new Vector2(getRightX(), getRightY());
    }
}
