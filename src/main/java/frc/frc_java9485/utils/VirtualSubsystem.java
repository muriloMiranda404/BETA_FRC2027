// Copyright (c) 2025-2026 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Adapted for RebuiltHyobots2026 (package renamed to frc.frc_java9485.utils).

package frc.frc_java9485.utils;

import java.util.ArrayList;
import java.util.List;


public abstract class VirtualSubsystem {
    private static final List<VirtualSubsystem> instances = new ArrayList<>();

    public VirtualSubsystem() {
        instances.add(this);
    }


    public abstract void periodic();


    public abstract void periodicAfterScheduler();


    public static void runAllPeriodic() {
        for (VirtualSubsystem instance : instances) {
            instance.periodic();
        }
    }


    public static void runAllPeriodicAfterScheduler() {
        for (VirtualSubsystem instance : instances) {
            instance.periodicAfterScheduler();
        }
    }
}
