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

import io.mavsdk.action.Action;
import io.mavsdk.camera.Camera;
import io.mavsdk.telemetry.Telemetry;
import net.opengis.swe.v20.DataBlock;
import net.opengis.swe.v20.DataComponent;
import net.opengis.swe.v20.DataEncoding;
import net.opengis.swe.v20.DataRecord;
import org.sensorhub.api.data.DataEvent;
import org.sensorhub.impl.sensor.AbstractSensorOutput;
import org.vast.swe.helper.GeoPosHelper;

import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.locks.ReentrantLock;


/**
 * UnmannedOutput specification and provider for {@link UnmannedSystem}.
 */
public class UnmannedOutput extends AbstractSensorOutput<UnmannedSystem> {
    static final String SENSOR_OUTPUT_NAME = "SensorOutput";
    static final String SENSOR_OUTPUT_LABEL = "UnmannedSystem UnmannedOutput";
    static final String SENSOR_OUTPUT_DESCRIPTION = "UnmannedSystem output data";

    private static final int MAX_NUM_TIMING_SAMPLES = 10;

    private final ArrayList<Double> intervalHistogram = new ArrayList<>(MAX_NUM_TIMING_SAMPLES);
    private final Object histogramLock = new Object();
    private final Object processingLock = new Object();

    private DataRecord dataRecord;
    private DataEncoding dataEncoding;

    static double destLatitude  = -35.355867;
    static double destLongitude = 149.169245;
    static float destAltitude = 30.0F;

    static double homeLatitude  = -35.362219;
    static double homeLongitude = 149.165082;
    static float homeAltitude = 30.0F;

    static double deltaSuccess =   0.000003;

    static Telemetry.Position    currentPosition = null;
    static Telemetry.EulerAngle  currentAttitude = null;
    static Telemetry.VelocityNed currentVelocity = null;
    static Telemetry.Imu         currentImu = null;
    private final ReentrantLock lock = new ReentrantLock();


    /**
     * Creates a new output for the sensor driver.
     *
     * @param parentSensor UnmannedSystem driver providing this output.
     */
    UnmannedOutput( UnmannedSystem parentSensor) {
        super(SENSOR_OUTPUT_NAME, parentSensor);
    }

    /**
     * Initializes the data structure for the output, defining the fields, their ordering, and data types.
     */
    void doInit() {
        // Get an instance of SWE Factory suitable to build components
        GeoPosHelper sweFactory = new GeoPosHelper();

        // Create the data record description
        dataRecord = sweFactory.createRecord()
                .name(SENSOR_OUTPUT_NAME)
                .label(SENSOR_OUTPUT_LABEL)
                .description(SENSOR_OUTPUT_DESCRIPTION)
                .addField("sampleTime", sweFactory.createTime()
                   .asSamplingTimeIsoUTC()
                   .label("Sample Time")
                   .description("Time of data collection"))
                .addField("Location", sweFactory.createLocationVectorLLA()
                   .label("Location")
                   .description("Location Latitude Longitude Altitude"))
                .addField("Orientation", sweFactory.createEulerOrientationYPR("deg"))
                .addField( "Velocity", sweFactory.createVelocityVectorNED("m/s")
                    .label("Velocity")
                    .description("Velocity"))
                .addField( "Acceleration", sweFactory.createAccelerationVector("m/s2")
                    .label("Acceleration")
                    .description("Acceleration measured by raw IMU"))
                .addField( "AngularVelocity", sweFactory.createAngularVelocityVector("rad/s")
                    .label("AngularVelocity")
                    .description("Angular velocity measured by gyroscopes from raw IMU"))
                .addField( "Temperature", sweFactory.createQuantity()
                    .label("Temperature")
                    .description("Temperature in degrees celsius"))
                .addField("MagneticField", sweFactory.createVector3()
                    .label("MagneticField")
                    .refFrame("FRD")
                    .build())
                .build();

        dataEncoding = sweFactory.newTextEncoding(",", "\n");
    }

    @Override
    public DataComponent getRecordDescription() {
        return dataRecord;
    }

    @Override
    public DataEncoding getRecommendedEncoding() {
        return dataEncoding;
    }

    @Override
    public double getAverageSamplingPeriod() {
        synchronized (histogramLock) {
            double sum = 0;
            for (double sample : intervalHistogram)
                sum += sample;

            return sum / intervalHistogram.size();
        }
    }


    public void setData( long timestamp ) {
        synchronized (processingLock) {
            DataBlock dataBlock = latestRecord == null ? dataRecord.createDataBlock() : latestRecord.renew();

            updateIntervalHistogram();

            lock.lock();
            try {
                int index = 0;
                // Populate the data block

                if ( currentPosition != null && currentVelocity != null && currentImu != null ) {
                    dataBlock.setDoubleValue(index++, timestamp / 1000d);

                    //Location
                    dataBlock.setDoubleValue(index++, currentPosition.getLatitudeDeg());
                    dataBlock.setDoubleValue(index++, currentPosition.getLongitudeDeg());
                    dataBlock.setDoubleValue(index++, currentPosition.getAbsoluteAltitudeM());

                    // Orientation
                    dataBlock.setDoubleValue(index++, currentAttitude.getYawDeg());
                    dataBlock.setDoubleValue(index++, currentAttitude.getPitchDeg());
                    dataBlock.setDoubleValue(index++, currentAttitude.getRollDeg());

                    //Velocity
                    dataBlock.setDoubleValue(index++, currentVelocity.getNorthMS());
                    dataBlock.setDoubleValue(index++, currentVelocity.getEastMS());
                    dataBlock.setDoubleValue(index++, currentVelocity.getDownMS());

                    //Acceleration
                    dataBlock.setDoubleValue(index++, currentImu.getAccelerationFrd().getForwardMS2());
                    dataBlock.setDoubleValue(index++, currentImu.getAccelerationFrd().getRightMS2());
                    dataBlock.setDoubleValue(index++, currentImu.getAccelerationFrd().getDownMS2());
                    //Angular Velocity
                    dataBlock.setDoubleValue(index++, currentImu.getAngularVelocityFrd().getForwardRadS());
                    dataBlock.setDoubleValue(index++, currentImu.getAngularVelocityFrd().getRightRadS());
                    dataBlock.setDoubleValue(index++, currentImu.getAngularVelocityFrd().getDownRadS());
                    //Temperature
                    dataBlock.setDoubleValue(index++, currentImu.getTemperatureDegc());
                    //Magnetic Field
                    dataBlock.setDoubleValue(index++, currentImu.getMagneticFieldFrd().getForwardGauss());
                    dataBlock.setDoubleValue(index++, currentImu.getMagneticFieldFrd().getRightGauss());
                    dataBlock.setDoubleValue(index++, currentImu.getMagneticFieldFrd().getDownGauss());
                }

            } finally {
                lock.unlock();
            }

            // Publish the data block
            latestRecord = dataBlock;
            latestRecordTime = timestamp;
            eventHandler.publish(new DataEvent(latestRecordTime, UnmannedOutput.this, dataBlock));
        }
    }


    /**
     * Updates the interval histogram with the time between the latest record and the current time
     * for calculating the average sampling period.
     */
    private void updateIntervalHistogram() {
        synchronized (histogramLock) {
            if (latestRecord != null && latestRecordTime != Long.MIN_VALUE) {
                long interval = System.currentTimeMillis() - latestRecordTime;
                intervalHistogram.add(interval / 1000d);

                if (intervalHistogram.size() > MAX_NUM_TIMING_SAMPLES) {
                    intervalHistogram.remove(0);
                }
            }
        }
    }


    public static void printHealth( io.mavsdk.System drone ) {
        drone.getTelemetry().getHealth()
                .subscribe(health -> {
                    System.out.println("Gyro online: " + health.getIsGyrometerCalibrationOk());
                    System.out.println("Accel online: " + health.getIsAccelerometerCalibrationOk());
                    System.out.println("Mag online: " + health.getIsMagnetometerCalibrationOk());
                    System.out.println("GPS online: " + health.getIsGlobalPositionOk());
                    // etc.
                });

    }

    public static void printParams( io.mavsdk.System drone ) {

        drone.getParam().getAllParams()
                .toObservable()
                .subscribe(
                        info -> {
                            info.getFloatParams().forEach( p -> {
                                System.out.println("FLOAT Param: " + p.getName() + " = " + p.getValue());
                            });
                            info.getIntParams().forEach( p -> {
                                System.out.println("INT Param: " + p.getName() + " = " + p.getValue());
                            });
                            info.getCustomParams().forEach( p -> {
                                System.out.println("Custom Param: " + p.getName() + " = " + p.getValue());
                            });
                        }
                );
    }

    public static void printVideoStreamInfo( io.mavsdk.System drone ) {
        drone.getCamera().getVideoStreamInfo()
                .toObservable()
                .subscribe(
                        info -> {
                            System.out.println("Stream URI: " + info.getVideoStreamInfo().getSettings().getUri());
                            System.out.println("Stream ID: " + info.getVideoStreamInfo().getStreamId());
                            System.out.println("Component ID: " + info.getComponentId());
                        },
                        error -> System.err.println("Failed to get video stream info: " + error)
                );

    }


    public void subscribeTelemetry( io.mavsdk.System drone ) {
        drone.getTelemetry().getPosition()
                .subscribe(
                        pos -> {
                            //System.out.println("MAVSDK: Lat: " + pos.getLatitudeDeg() + ", Lon: " + pos.getLongitudeDeg());

                            lock.lock();
                            try {
                                currentPosition = pos;
                            } finally {
                                lock.unlock();
                            }

                            setData(System.currentTimeMillis());
                        },
                        err -> System.err.println("MAVSDK: Position error: " + err)
                );

        drone.getTelemetry().getVelocityNed()
                .subscribe(
                        vel -> {
                            //System.out.println("MAVSDK: Velocity E:" + vel.getEastMS() + " M/S, N:" + vel.getNorthMS() + " M/S");

                            lock.lock();
                            try {
                                currentVelocity = vel;
                            } finally {
                                lock.unlock();
                            }

                            setData(System.currentTimeMillis());
                        },
                        err -> System.err.println("MAVSDK: Position error: " + err)
                );


        drone.getTelemetry().setRateRawImu(0.5) // Hz
                .subscribe(() -> System.out.println("Rate set."),
                        err -> System.err.println("Failed: " + err.getMessage()));

        drone.getTelemetry().getRawImu()
                .subscribe(imu -> {

                    lock.lock();
                    try {
                        currentImu = imu;
                    } finally {
                        lock.unlock();
                    }

                    setData(System.currentTimeMillis());

                    //System.out.println("Accel Forward: " + imu.getAccelerationFrd().getForwardMS2() + " m/s^2");
                    //System.out.println("Accel Right: " + imu.getAccelerationFrd().getRightMS2() + " m/s^2");
                    //System.out.println("Accel Down: " + imu.getAccelerationFrd().getDownMS2() + " m/s^2");

                    //System.out.println("Gyro Forward: " + imu.getAngularVelocityFrd().getForwardRadS() + " rad");
                    //System.out.println("Gyro Right: " + imu.getAngularVelocityFrd().getRightRadS() + " rad");
                    //System.out.println("Gyro Down: " + imu.getAngularVelocityFrd().getDownRadS() + " rad");

                    //System.out.println("Temperature: " + imu.getTemperatureDegc() + " degC");

                    //System.out.println("Magnetic Field Forward: " + imu.getMagneticFieldFrd().getForwardGauss() + " Gauss");
                    //System.out.println("Magnetic Field Right: " + imu.getMagneticFieldFrd().getRightGauss() + " Gauss");
                    //System.out.println("Magnetic Field Down: " + imu.getMagneticFieldFrd().getDownGauss() + " Gauss");
                });

        drone.getTelemetry().getAttitudeEuler()
                .subscribe(attitude -> {
                    lock.lock();
                    try {
                        currentAttitude = attitude;
                    } finally {
                        lock.unlock();
                    }

                    setData(System.currentTimeMillis());
                });

    }

    public static void downloadLog( io.mavsdk.System drone ) {

        CountDownLatch latch = new CountDownLatch(1);

        drone.getLogFiles().getEntries()
                .toFlowable()
                .map(entries -> entries.get(entries.size() - 1))
                .flatMap(lastEntry ->
                        drone.getLogFiles().downloadLogFile(lastEntry, "./example_log.ulg"))
                .map(progressData -> Math.round(progressData.getProgress() * 100))
                .filter(progressPercent -> progressPercent % 5 == 0)
                .distinctUntilChanged()
                .subscribe(
                        progressPercent -> {
                            System.out.println("Progress: " + progressPercent + "%");
                        },
                        throwable -> {
                            System.out.println("Error: " + throwable.getMessage());
                            latch.countDown();
                        },
                        () -> {
                            System.out.println("Successfully downloaded last log!");
                            latch.countDown();
                        });

        try {
            latch.await();
        } catch (InterruptedException ignored) {
            // This is expected
        }
    }

    public static void printTransponderInfo( io.mavsdk.System drone ) {
        drone.getTransponder().getTransponder()
                .subscribe(
                        transponder -> {
                            System.out.println("CallSign: " + transponder.getCallsign());
                            System.out.println("Squawk: " + transponder.getSquawk());
                            System.out.println("Icao Address: " + transponder.getIcaoAddress());
                        }
                );
    }

    public static void printDroneInfo( io.mavsdk.System drone ) {

//        drone.getCamera().getCameraList()
//            .subscribe( cameraListUpdate -> {
//                 System.out.println("Camera 0 Model: " + cameraListUpdate.getCameras().get(0).getModelName());
//             }, throwable -> {
//                 System.out.println("Failed to get camera list: " + throwable.getMessage());
//             });

        drone.getInfo().getVersion()
                .flatMap(version -> {
                    System.out.println("Flight software version: " +
                            version.getFlightSwMajor() + "." +
                            version.getFlightSwMinor() + "." +
                            version.getFlightSwPatch());

                    // Optionally infer autopilot from version or Git hash
                    return drone.getInfo().getIdentification();
                })
                .subscribe(identification -> {
                    System.out.println("Hardware UID: " + identification.getHardwareUid());
                }, throwable -> {
                    System.err.println("Failed to get info: " + throwable.getMessage());
                });


        // Get video stream info after arming
        drone.getCamera().getVideoStreamInfo()
                .subscribe(videoStreamUpdate -> {
                    Camera.VideoStreamInfo info = videoStreamUpdate.getVideoStreamInfo();
                    System.out.println("Stream URI: " + info.getSettings().getUri());
                    System.out.println("Rotation: " + info.getSettings().getRotationDeg());
                    System.out.println("BitRate: " + info.getSettings().getBitRateBS());
                    System.out.println("FPS: " + info.getSettings().getFrameRateHz());
                    System.out.println("Resolution: " + info.getSettings().getHorizontalResolutionPix() + "x" +
                            info.getSettings().getVerticalResolutionPix());
                    System.out.println("Status: " + info.getStatus());
                }, throwable -> {
                    System.out.println("Failed to get video stream info: " + throwable.getMessage());
                });


    }


    private static void sendMission( io.mavsdk.System drone ) {

        drone.getAction().arm()
                .doOnComplete(() -> {

                    System.out.println("Arming...");

                    //mission

                })
                .doOnError(throwable -> {

                    System.out.println("Failed to arm: " + ((Action.ActionException) throwable).getCode());

                });
    }

}
