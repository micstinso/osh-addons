/***************************** BEGIN LICENSE BLOCK ***************************
 The contents of this file are subject to the Mozilla Public License, v. 2.0.
 If a copy of the MPL was not distributed with this file, You can obtain one
 at http://mozilla.org/MPL/2.0/.

 Software distributed under the License is distributed on an "AS IS" basis,
 WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
 for the specific language governing rights and limitations under the License.

 Copyright (C) 2020-2025 Botts Innovative Research, Inc. All Rights Reserved.
 ******************************* END LICENSE BLOCK ***************************/
package org.sensorhub.impl.comm.mavlink2;

import org.sensorhub.api.common.SensorHubException;
import org.sensorhub.impl.sensor.AbstractSensorModule;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Driver implementation for the sensor.
 * <p>
 * This class is responsible for providing sensor information, managing output registration,
 * and performing initialization and shutdown for the driver and its outputs.
 */
public class UnmannedSystem extends AbstractSensorModule<UnmannedConfig> {
    static final String UID_PREFIX = "urn:osh:driver:mavlink2:";
    static final String XML_PREFIX = "MAVLINK2_DRIVER_";

    private static final Logger logger = LoggerFactory.getLogger(UnmannedSystem.class);

    UnmannedOutput output;
    Thread processingThread;
    volatile boolean doProcessing = true;

    UnmannedControlTakeoff unmannedControlTakeoff;
    UnmannedControlLocation unmannedControlLocation;
    UnmannedControlLanding unmannedControlLanding;
    UnmannedControlOffboard unmannedControlOffboard;

    @Override
    public void doInit() throws SensorHubException {
        super.doInit();

        // Generate identifiers
        generateUniqueID(UID_PREFIX, config.serialNumber);
        generateXmlID(XML_PREFIX, config.serialNumber);

        // Create and initialize output
        output = new UnmannedOutput(this);
        addOutput(output, false);

        // add Lat/Lon/Alt control stream
        this.unmannedControlTakeoff = new UnmannedControlTakeoff(this);
        addControlInput(this.unmannedControlTakeoff);
        unmannedControlTakeoff.init();

        this.unmannedControlLocation = new UnmannedControlLocation(this);
        addControlInput(this.unmannedControlLocation);
        unmannedControlLocation.init();

        this.unmannedControlLanding = new UnmannedControlLanding(this);
        addControlInput(this.unmannedControlLanding);
        unmannedControlLanding.init();

        this.unmannedControlOffboard = new UnmannedControlOffboard(this);
        addControlInput(this.unmannedControlOffboard);
        unmannedControlOffboard.init();

        output.doInit(unmannedControlLocation,unmannedControlLanding, unmannedControlTakeoff);
    }

    @Override
    public void doStart() throws SensorHubException {
        super.doStart();
        //startProcessing();
    }

    @Override
    public void doStop() throws SensorHubException {
        super.doStop();
        stopProcessing();
    }

    @Override
    public boolean isConnected() {
        return processingThread != null && processingThread.isAlive();
    }

    /**
     * Starts the data processing thread.
     * <p>
     * This method simulates sensor data collection and processing by generating data samples at regular intervals.
     */
    public void startProcessing() {
        doProcessing = true;

        processingThread = new Thread(() -> {
            while (doProcessing) {
                // Simulate data collection and processing
                //output.setData(System.currentTimeMillis(), "Sample Data");

                // Simulate a delay between data samples
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        processingThread.start();
    }

    /**
     * Signals the processing thread to stop.
     */
    public void stopProcessing() {
        doProcessing = false;
    }
}
