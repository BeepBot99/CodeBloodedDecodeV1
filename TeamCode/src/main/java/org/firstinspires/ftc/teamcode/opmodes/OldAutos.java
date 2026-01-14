package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Old Autos", preselectTeleOp = "TeleOp")
public class OldAutos extends SelectableOpMode {
    public OldAutos() {
        super("Old Autos", select -> {
            select.folder("Red", red -> {
                red.folder("Close", close -> {
                    close.add("Leave", RedCloseLeave::new);
                    close.add("9", RedClose9::new);
                    close.add("12", RedClose12::new);
                });
                red.folder("Far", far -> {
                    far.add("Leave", RedFarLeave::new);
                });
            });
            select.folder("Blue", blue -> {
                blue.folder("Close", close -> {
                    close.add("Leave", BlueCloseLeave::new);
                    close.add("9", BlueClose9::new);
                    close.add("12", BlueClose12::new);
                });
                blue.folder("Far", far -> {
                    far.add("Leave", BlueFarLeave::new);
                });
            });
        });
    }
}
