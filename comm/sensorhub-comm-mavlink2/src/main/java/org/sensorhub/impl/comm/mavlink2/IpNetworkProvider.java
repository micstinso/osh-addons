/***************************** BEGIN LICENSE BLOCK ***************************

The contents of this file are subject to the Mozilla Public License, v. 2.0.
If a copy of the MPL was not distributed with this file, You can obtain one
at http://mozilla.org/MPL/2.0/.

Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
for the specific language governing rights and limitations under the License.
 
Copyright (C) 2012-2015 Sensia Software LLC. All Rights Reserved.
 
******************************* END LICENSE BLOCK ***************************/

package org.sensorhub.impl.comm.mavlink2;

import org.sensorhub.api.comm.ICommNetwork.NetworkType;
import org.sensorhub.api.comm.INetworkProvider;
import org.sensorhub.api.module.IModule;
import org.sensorhub.api.module.ModuleConfig;


public class IpNetworkProvider implements INetworkProvider
{
    
    @Override
    public String getModuleName()
    {
        return "MAVLink2";
    }

    @Override
    public String getModuleDescription()
    {
        return "Network Service for MAVLink2 Support";
    }

    @Override
    public String getModuleVersion()
    {
        return "0.1";
    }

    @Override
    public String getProviderName()
    {
        return "Botts Innovative Research";
    }

    @Override
    public Class<? extends IModule<?>> getModuleClass()
    {
        return IpCommNetwork.class;
    }

    @Override
    public Class<? extends ModuleConfig> getModuleConfigClass()
    {
        return IpNetworkConfig.class;
    }

    @Override
    public NetworkType getNetworkType()
    {
        return NetworkType.IP;
    }
}
