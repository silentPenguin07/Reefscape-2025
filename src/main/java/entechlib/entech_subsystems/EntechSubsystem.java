/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package entechlib.entech_subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base class for better subsystem creation.
 */
public abstract class EntechSubsystem extends SubsystemBase {

    protected EntechSubsystem() {
    }

    /**
     * Initializes the subsystem. Be sure to wrap this in an if isEnabled check.
     */
    public abstract void initialize();

    /**
     * Used to indicate that the subsystem is enabled or not.
     * 
     * 
     * @return enabled
     */
    public abstract boolean isEnabled();
}
