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
import io.reactivex.Completable;
import net.opengis.swe.v20.*;
import org.sensorhub.api.command.CommandException;
import org.sensorhub.impl.sensor.AbstractSensorControl;
import org.vast.swe.SWEHelper;
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

    private io.mavsdk.System system = null;

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

        /*
                .addField("freq", sml.createQuantity()
                    .definition(SWEHelper.getQudtUri("Frequency"))
                    .label("Frequency")
                    .uomCode(freqUnit)
                    .build())
         */

        commandDataStruct = factory.createRecord()
                .name(SENSOR_CONTROL_NAME)
                .label(SENSOR_CONTROL_LABEL)
                .description(SENSOR_CONTROL_DESCRIPTION)
                .addField( "locationVectorLLA", factory.createVector()
                        .addCoordinate("Latitude", factory.createQuantity()
                                .uom("deg"))
                        .addCoordinate("Longitude", factory.createQuantity()
                                .uom("deg"))
                        .addCoordinate("AltitudeAGL", factory.createQuantity()
                                .uom("m")
                                .value(30)))
                .addField( "returnToStart", factory.createBoolean().value(false))
                .addField( "hoverSeconds", factory.createCount().value(0))
                .build();
    }


    @Override
    protected boolean execCommand(DataBlock command) throws CommandException {

        double latitude = command.getDoubleValue(0);
        double longitude = command.getDoubleValue(1);
        double altitude = command.getDoubleValue(2);
        boolean returnToStart = command.getBooleanValue(3);
        long hoverSeconds = command.getLongValue(4);

        System.out.println("Command received - Lat: " + latitude + " Lon: " + longitude + " Alt: " + altitude );

        if ( system != null ) {
            sendToLocation( latitude, longitude, altitude, returnToStart, hoverSeconds );
        } else {
            throw new CommandException("Unmanned System not initialized");
        }

        return true;
    }


    private void sendToLocation( double latitudeParam,
                                 double longitudeParam,
                                 double altAglParam,
                                 boolean returnHomeParam,
                                 long hoverSecondsParam ) {

        System.out.println("Preparing to send to location...");

        float absoluteMsl = system.getTelemetry().getPosition().blockingFirst().getAbsoluteAltitudeM();
        System.out.println("absolute MSL (m): " + absoluteMsl);

        float relativeAltitude = system.getTelemetry().getPosition().blockingFirst().getRelativeAltitudeM();
        System.out.println("current Relative MSL (m): " + relativeAltitude);

        float terrainOffset = absoluteMsl - relativeAltitude;
        System.out.println("terrain offset: " + terrainOffset);

        // We can use the last
        // known relative altitude to set an approximate MSL. This will sometimes not be adequate as the
        // terrain will change. For now it's good for a quick demo, but we'll want to eventually
        // constantly adjust based on terrain altitude.
        float altMsl = (float)altAglParam + terrainOffset;
        System.out.println("setting altitude MSL to: " + altMsl);

        double homeLat = system.getTelemetry().getPosition().blockingFirst().getLatitudeDeg();
        double homeLon = system.getTelemetry().getPosition().blockingFirst().getLongitudeDeg();

        system.getAction().gotoLocation(latitudeParam, longitudeParam, altMsl, 45.0F)
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
                        .delay(hoverSecondsParam, TimeUnit.SECONDS) //hover if the value is marked
                        .andThen( Completable.fromAction(() -> {

                            //return home if the value is checked
                            if ( returnHomeParam ) {
                                system.getAction().gotoLocation(homeLat, homeLon, altMsl, 45.0F)
                                        .doOnComplete(() -> {
                                            System.out.println("Moving back to home");
                                        })
                                        .doOnError(throwable -> {
                                            System.out.println("Failed to go home: " + ((Action.ActionException) throwable).getCode());
                                        })
                                        .subscribe();
                            }
                        }))
                )
                .subscribe();

    }


    public void stop() {
        // TODO Auto-generated method stub
    }
}

