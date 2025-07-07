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
import net.opengis.swe.v20.*;
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
public class UnmannedControlLocation extends AbstractSensorControl<UnmannedSystem>
{
    private DataRecord commandDataStruct;

    /**
     * Name of the control
     */
    private static final String SENSOR_CONTROL_NAME = "UnmannedControlLocation";

    /**
     * Label for the control
     */
    private static final String SENSOR_CONTROL_LABEL = "Location Control";

    /**
     * Control description
     */
    private static final String SENSOR_CONTROL_DESCRIPTION =
            "Interfaces with MAVLINK and OSH to effectuate control over the platform";

    /**
     * ROS Node name assigned at creation
     */
    private static final String NODE_NAME_STR = "/SensorHub/spot/location_control";

    private io.mavsdk.System system = new io.mavsdk.System();

    static double deltaSuccess =   0.000003; //distance from lat/lon to determine success

    public UnmannedControlLocation( UnmannedSystem parentSensor) {
        super("mavControl", parentSensor);
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
                .addField( "locationVectorLLA", factory.createVector()
                        .addCoordinate("Latitude", factory.createQuantity())
                        .addCoordinate("Longitude", factory.createQuantity())
                        .addCoordinate("AltitudeAGL", factory.createQuantity()))
                .build();
    }


    @Override
    protected boolean execCommand(DataBlock command) throws CommandException {

        double latitude = command.getDoubleValue(0);
        double longitude = command.getDoubleValue(1);
        double altitude = command.getDoubleValue(2);

        System.out.println("Command received - Lat: " + latitude + " Lon: " + longitude + " Alt: " + altitude );

        if ( system != null ) {
            sendToLocation( latitude, longitude, altitude );
        }

        return true;
    }


    private void sendToLocation( double latitudeParam,
                                 double longitudeParam,
                                 double altAglParam ) {

        System.out.println("Setting up scenario...");

        CountDownLatch latch = new CountDownLatch(1);

        //Ensure drone is armed first and at takeoff altitude then move to location
/*        system.getTelemetry().getArmed()
                .filter(armed -> armed)
                .firstElement()
                .ignoreElement()
                .andThen(system.getTelemetry().getPosition()
                        .filter(pos -> pos.getRelativeAltitudeM() >= altAglParam )
                        .firstElement()
                        .ignoreElement()
                )
                .delay(1, TimeUnit.SECONDS)
                .andThen(*/
        system.getAction().gotoLocation(latitudeParam, longitudeParam,
                                (float)altAglParam + system.getTelemetry().getPosition().blockingFirst().getAbsoluteAltitudeM(),
                                45.0F)
                .doOnComplete( () -> {

                    System.out.println("Moving to target location");

                })
                .doOnError( throwable -> {

                    System.out.println("Failed to go to target: " + ((Action.ActionException) throwable).getCode());

                })
                .andThen(system.getTelemetry().getPosition()
                        .filter(pos -> (abs(pos.getLatitudeDeg() - latitudeParam) <= deltaSuccess && abs(pos.getLongitudeDeg() - longitudeParam) <= deltaSuccess))
                        .firstElement()
                        .ignoreElement()
                )
                .subscribe(latch::countDown, throwable -> {
                    System.out.println("Reached target location");
                    latch.countDown();
                });

    }


    public void stop() {
        // TODO Auto-generated method stub
    }
}

