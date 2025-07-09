/***************************** BEGIN LICENSE BLOCK ***************************

 The contents of this file are subject to the Mozilla Public License, v. 2.0.
 If a copy of the MPL was not distributed with this file, You can obtain one
 at http://mozilla.org/MPL/2.0/.

 Software distributed under the License is distributed on an "AS IS" basis,
 WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
 for the specific language governing rights and limitations under the License.

 The Initial Developer is Botts Innovative Research Inc. Portions created by the Initial
 Developer are Copyright (C) 2025 the Initial Developer. All Rights Reserved.

 ******************************* END LICENSE BLOCK ***************************/


package org.sensorhub.impl.comm.mavlink2;

import io.mavsdk.action.Action;
import net.opengis.swe.v20.DataBlock;
import net.opengis.swe.v20.DataComponent;
import net.opengis.swe.v20.DataRecord;
import org.sensorhub.api.command.CommandException;
import org.sensorhub.impl.sensor.AbstractSensorControl;
import org.vast.swe.helper.GeoPosHelper;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import static java.lang.Math.abs;

/**
 * <p>
 * This particular class provides control stream  capabilities
 * </p>
 *
 * @author Michael Stinson
 * @since Jul 2025
 */
public class UnmannedControlTakeoff extends AbstractSensorControl<UnmannedSystem>
{
    private DataRecord commandDataStruct;

    /**
     * Name of the control
     */
    private static final String SENSOR_CONTROL_NAME = "UnmannedControlTakeoff";

    /**
     * Label for the control
     */
    private static final String SENSOR_CONTROL_LABEL = "Takeoff Control";

    /**
     * Control description
     */
    private static final String SENSOR_CONTROL_DESCRIPTION =
            "Interfaces with MAVLINK and OSH to effectuate takeoff control over the platform";

    /**
     * ROS Node name assigned at creation
     */
    private static final String NODE_NAME_STR = "/SensorHub/spot/takeoff_control";

    private io.mavsdk.System system = null;

    static double deltaSuccess =   0.000003; //distance from lat/lon to determine success

    public UnmannedControlTakeoff( UnmannedSystem parentSensor) {
        super("mavTakeoffControl", parentSensor);
    }


    @Override
    public DataComponent getCommandDescription() {
        return commandDataStruct;
    }

    public void setSystem( io.mavsdk.System systemParam ) {
        system = systemParam;
    }

    public void init() {

        GeoPosHelper factory = new GeoPosHelper();

        commandDataStruct = factory.createRecord()
                .name(SENSOR_CONTROL_NAME)
                .label(SENSOR_CONTROL_LABEL)
                .description(SENSOR_CONTROL_DESCRIPTION)
                .addField("TakeoffAltitudeAGL", factory.createQuantity())
                .build();
    }


    @Override
    protected boolean execCommand(DataBlock command) throws CommandException {

        double altitude = command.getDoubleValue(0);

        System.out.println("Command received - Alt: " + altitude );

        if ( system != null ) {
            takeOff( altitude );
        } else {
            throw new CommandException("Unmanned System not initialized");
        }

        return true;
    }


    private void takeOff( double altAglParam ) {

        System.out.println("Setting up scenario...");

        System.out.println("Setting takeoff altitude AGL: " + altAglParam);

        system.getAction().arm()
                .doOnComplete(() -> {

                    System.out.println("Arming...");

                })
                .doOnError(throwable -> {

                    System.out.println("Failed to arm: " + throwable.getMessage());

                })
                .andThen(system.getAction().setTakeoffAltitude((float)altAglParam))
                .andThen(system.getAction().takeoff()
                        .doOnComplete(() -> {

                            System.out.println("Taking off...");

                        })
                        .doOnError(throwable -> {

                            System.out.println("Failed to take off");

                        }))
                .andThen(system.getTelemetry().getPosition()
                        .filter(pos -> pos.getRelativeAltitudeM() >= altAglParam)
                        .firstElement()
                        .ignoreElement()
                )
                .subscribe(() -> {
                    System.out.println("Reached takeoff altitude");
                    },
                        throwable -> System.out.println("Failed")
                );

    }


    public void stop() {
        // TODO Auto-generated method stub
    }
}

