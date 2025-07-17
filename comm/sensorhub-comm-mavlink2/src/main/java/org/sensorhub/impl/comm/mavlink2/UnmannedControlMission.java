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
import io.mavsdk.mission.Mission;
import io.reactivex.Completable;
import net.opengis.swe.v20.DataBlock;
import net.opengis.swe.v20.DataComponent;
import net.opengis.swe.v20.DataRecord;
import org.sensorhub.api.command.CommandException;
import org.sensorhub.impl.comm.mavlink2.config.MissionConfig;
import org.sensorhub.impl.comm.mavlink2.config.PointConfig;
import org.sensorhub.impl.sensor.AbstractSensorControl;
import org.vast.swe.helper.GeoPosHelper;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

/**
 * <p>
 * This particular class provides control stream  capabilities
 * </p>
 *
 * @author Michael Stinson
 * @since Jul 2025
 */
public class UnmannedControlMission extends AbstractSensorControl<UnmannedSystem>
{
    private DataRecord commandDataStruct;

    /**
     * Name of the control
     */
    private static final String SENSOR_CONTROL_NAME = "UnmannedControlMission";

    /**
     * Label for the control
     */
    private static final String SENSOR_CONTROL_LABEL = "Mission Control";

    /**
     * Control description
     */
    private static final String SENSOR_CONTROL_DESCRIPTION =
            "Interfaces with MAVLINK and OSH to effectuate mission control over the platform";

    /**
     * ROS Node name assigned at creation
     */
    private static final String NODE_NAME_STR = "/SensorHub/spot/mission_control";

    private io.mavsdk.System system = null;

    private UnmannedControlLocation locationControl = null;

    static double deltaSuccess =   0.000003; //distance from lat/lon to determine success

    public UnmannedControlMission( UnmannedSystem parentSensor) {
        super("mavMissionControl", parentSensor);
    }


    @Override
    public DataComponent getCommandDescription() {
        return commandDataStruct;
    }

    public void setSystem( io.mavsdk.System systemParam ) {
        system = systemParam;
    }

    public void setLocationControl( UnmannedControlLocation locationControlParam ) {
        locationControl = locationControlParam;
    }

    public void init() {

        GeoPosHelper factory = new GeoPosHelper();

        commandDataStruct = factory.createRecord()
                .name(SENSOR_CONTROL_NAME)
                .label(SENSOR_CONTROL_LABEL)
                .description(SENSOR_CONTROL_DESCRIPTION)
                .addField("mission", factory.createCount().value(1))
                .addField( "returnToStart", factory.createBoolean().value(false))
                .build();
    }


    @Override
    protected boolean execCommand(DataBlock command) throws CommandException {
        int missionNo = command.getIntValue(0);
        boolean returnToStart = command.getBooleanValue(1);

        if ( system != null ) {
            mission(missionNo, returnToStart);
        } else {
            throw new CommandException("Unmanned System not initialized");
        }

        return true;
    }


    //Get the configuration
    //build a mission for the drone
    //send the mission via mavlink


    /**
     * Currently missions through MAVSDK appear to not work via ArduPilot SITL. For now
     * lets control the mission directly
     * @param missionNumberParam
     */
    private void mission( int missionNumberParam, boolean returnToStartParam ) {

        var sensor = this.getParentProducer();
        var config = sensor.getConfiguration();
        MissionConfig mission = config.missions.get(missionNumberParam-1);

        List<Mission.MissionItem> finalMission = new ArrayList<>();

        double homeLat = system.getTelemetry().getPosition().blockingFirst().getLatitudeDeg();
        double homeLon = system.getTelemetry().getPosition().blockingFirst().getLongitudeDeg();
        double homeAGL = mission.points.get(0).altitudeAGL;

        double altMsl = locationControl.getAltMsl((float)homeAGL);

        Completable missionChain = Completable.concat(
                mission.points.stream()
                        //.map(point -> locationControl.goLocation(point.latitude, point.longitude, false, (long) point.hoverSeconds, (float)altMsl )
                        .map(point -> locationControl.goLocation(point.latitude,point.longitude,false,(long)point.hoverSeconds,(float)altMsl,homeLat, homeLon)
                                .doOnError(throwable -> {
                                    System.out.println("Failed to go to mission target point: " + ((Action.ActionException) throwable).getCode());
                                })
                                .andThen(system.getTelemetry().getPosition()
                                        .filter(pos -> (abs(pos.getLatitudeDeg() - point.latitude) <= deltaSuccess &&
                                                abs(pos.getLongitudeDeg() - point.longitude) <= deltaSuccess))
                                        .firstElement()
                                        .ignoreElement())
                        )
                        .toList()
        );

        missionChain.subscribe(
                () -> {
                    System.out.println("Mission complete");
                    if ( returnToStartParam ) {
                        locationControl.goLocation(homeLat, homeLon,false,0,(float)altMsl,homeLat, homeLon)
                                .subscribe();
                    }
                },
                throwable -> System.out.println("Mission failed: " + throwable)
        );

        //Re-enable when we get direct ArduPilot SITL missions working.
        //it appears to now show UNSUPPORTED. Needs further investigation.
//        System.out.println("Running mission...");
//
//        Mission.MissionPlan missionPlan = new Mission.MissionPlan(finalMission);
//        System.out.println("About to upload " + finalMission.size() + " mission items");
//
//        CountDownLatch latch = new CountDownLatch(1);
//        system.getMission()
//                .setReturnToLaunchAfterMission(true)
//                .andThen(system.getMission().uploadMission(missionPlan)
//                        .doOnComplete(() -> System.out.println("Upload succeeded"))
//                        .doOnError(e -> {
//                           System.out.println("error: " + e.getMessage() + " ");
//                           e.printStackTrace();
//                        }))
//                .andThen((CompletableSource) cs -> latch.countDown())
//                .subscribe();
//
//        try {
//            latch.await();
//        } catch (InterruptedException ignored) {
//            // This is expected
//        }
    }


    public void stop() {
        // TODO Auto-generated method stub
    }


    /**
     * Currently missions through MAVSDK appear to not work via ArduPilot SITL. For now
     * lets control the mission directly
     * @param latitudeDeg
     * @param longitudeDeg
     * @param hoverSecondsParam
     * @return
     */
    public static Mission.MissionItem generateMissionItem( double latitudeDeg, double longitudeDeg, double hoverSecondsParam ) {

        //MissionItem(java.lang.Double latitudeDeg,
        //            java.lang.Double longitudeDeg,
        //            java.lang.Float relativeAltitudeM,
        //            java.lang.Float speedMS,
        //            java.lang.Boolean isFlyThrough,
        //            java.lang.Float gimbalPitchDeg,
        //            java.lang.Float gimbalYawDeg,
        //            io.mavsdk.mission.Mission.MissionItem.CameraAction cameraAction,
        //            java.lang.Float loiterTimeS,
        //            java.lang.Double cameraPhotoIntervalS,
        //            java.lang.Float acceptanceRadiusM,
        //            java.lang.Float yawDeg,
        //            java.lang.Float cameraPhotoDistanceM,
        //            io.mavsdk.mission.Mission.MissionItem.VehicleAction vehicleAction)

        return new Mission.MissionItem(
                latitudeDeg,
                longitudeDeg,
                20f,
                1.0f,
                true,
                0f,
                0f,
                Mission.MissionItem.CameraAction.NONE,
                (float)hoverSecondsParam,
                1.0,
                0f,
                0f,
                0f,
                Mission.MissionItem.VehicleAction.NONE);
    }
}

