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

import net.opengis.swe.v20.*;
import org.sensorhub.api.command.CommandException;
import org.sensorhub.impl.sensor.AbstractSensorControl;
import org.vast.swe.SWEHelper;
import org.vast.swe.helper.GeoPosHelper;

/**
 * <p>
 * This particular class provides control stream  capabilities
 * </p>
 *
 * @author Michael Stinson
 * @since Jul 2025
 */
public class UnmannedControl extends AbstractSensorControl<UnmannedSystem>
{
    private DataRecord commandDataStruct;

    /**
     * Name of the control
     */
    private static final String SENSOR_CONTROL_NAME = "UnmannedControl";

    /**
     * Label for the control
     */
    private static final String SENSOR_CONTROL_LABEL = "Unmanned System Controls";

    /**
     * Control description
     */
    private static final String SENSOR_CONTROL_DESCRIPTION =
            "Interfaces with MAVLINK and OSH to effectuate control over the platform";

    /**
     * ROS Node name assigned at creation
     */
    private static final String NODE_NAME_STR = "/SensorHub/spot/pose_control";


    public UnmannedControl( UnmannedSystem parentSensor) {
        super("mavControl", parentSensor);
    }


    @Override
    public DataComponent getCommandDescription() {
        return commandDataStruct;
    }

    private enum PoseCommands {
        LOCATION_LLA
    }

    public void init() {

        GeoPosHelper factory = new GeoPosHelper();

        commandDataStruct = factory.createRecord()
                .name(SENSOR_CONTROL_NAME)
                .label(SENSOR_CONTROL_LABEL)
                .description(SENSOR_CONTROL_DESCRIPTION)
                .definition(SWEHelper.getPropertyUri("pose_controls"))
                .addField("poseCommand", factory.createChoice()
                        .label("UAS Command")
                        .description("Commands to control the location of the platform")
                        .definition(SWEHelper.getPropertyUri("location_command"))
                        .addItem(PoseCommands.LOCATION_LLA.name(), factory.createRecord()
                                .label("Location")
                                .description("Latitude Longitude Altitude")
                                .definition(SWEHelper.getPropertyUri("posed_stand"))
                                .addField("locationVectorLLA", factory.createLocationVectorLLA())
                                .build())
                        .build())
                .build();

    }


    @Override
    protected boolean execCommand(DataBlock command) throws CommandException {
        return true;
    }


    public void stop() {
        // TODO Auto-generated method stub
    }
}

