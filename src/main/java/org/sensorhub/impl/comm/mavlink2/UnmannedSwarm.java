package org.sensorhub.impl.comm.mavlink2;

import org.sensorhub.impl.sensor.SensorSystem;

public class UnmannedSwarm extends SensorSystem {

    public UnmannedSwarm() {
        super();

        getMembers();

        // Add control input that distributes LLA command among members
        addControlInput();
    }

}
