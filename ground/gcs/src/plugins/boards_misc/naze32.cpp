/**
 ******************************************************************************
 * @file       naze32.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_TauLabsPlugin Tau Labs boards support Plugin
 * @{
 * @brief Plugin to support boards by the Tau Labs project
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "naze32.h"

#include <uavobjectmanager.h>
#include "uavobjectutil/uavobjectutilmanager.h"
#include <extensionsystem/pluginmanager.h>

#include "hwnaze32.h"

/**
 * @brief Naze32::Naze32
 *  This is the Naze32 board definition
 */
Naze32::Naze32(void)
{
    boardType = 0x94;

    // Define the bank of channels that are connected to a given timer
    channelBanks.resize(2);
    channelBanks[0] = QVector<int> () << 1 << 2;
    channelBanks[1] = QVector<int> () << 3 << 4 << 5 << 6;
}

Naze32::~Naze32()
{

}


QString Naze32::shortName()
{
    return QString("Naze32");
}

QString Naze32::boardDescription()
{
    return QString("The Naze32 board");
}

//! Return which capabilities this board has
bool Naze32::queryCapabilities(BoardCapabilities capability)
{
    switch(capability) {
    case BOARD_CAPABILITIES_GYROS:
        return true;
    case BOARD_CAPABILITIES_ACCELS:
        return true;
    case BOARD_CAPABILITIES_MAGS:
        return true;
    case BOARD_CAPABILITIES_BAROS:
        return true;
    case BOARD_CAPABILITIES_RADIO:
        return false;
    }
    return false;
}


/**
 * @brief Naze32::getSupportedProtocols
 *  TODO: this is just a stub, we'll need to extend this a lot with multi protocol support
 * @return
 */
QStringList Naze32::getSupportedProtocols()
{

    return QStringList("uavtalk");
}

QPixmap Naze32::getBoardPicture()
{
    return QPixmap(":/misc/images/naze32.png");
}

QString Naze32::getHwUAVO()
{
    return "HwNaze32";
}

//! Determine if this board supports configuring the receiver
bool Naze32::isInputConfigurationSupported()
{
    return true;
}

/**
 * Configure the board to use a receiver input type on a port number
 * @param type the type of receiver to use
 * @param port_num which input port to configure (board specific numbering)
 * @return true if successfully configured or false otherwise
 */
bool Naze32::setInputOnPort(enum InputType type, int port_num)
{
    if (port_num != 0)
        return false;

    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *uavoManager = pm->getObject<UAVObjectManager>();
    HwNaze32 *hwNaze32 = HwNaze32::GetInstance(uavoManager);
    Q_ASSERT(hwNaze32);
    if (!hwNaze32)
        return false;

    HwNaze32::DataFields settings = hwNaze32->getData();

    switch(type) {
    case INPUT_TYPE_PWM:
        settings.RcvrPort = HwNaze32::RCVRPORT_PWM;
        break;
	case INPUT_TYPE_PPM:
        settings.RcvrPort = HwNaze32::RCVRPORT_PPM;
        break;
    case INPUT_TYPE_SBUS:
        settings.RcvrPort = HwNaze32::RCVRPORT_SBUS;
        break;
    case INPUT_TYPE_DSM:
        settings.RcvrPort = HwNaze32::RCVRPORT_DSM;
        break;
    default:
        return false;
    }

    // Apply these changes
    hwNaze32->setData(settings);

    return true;
}

/**
 * @brief Naze32::getInputOnPort fetch the currently selected input type
 * @param port_num the port number to query (must be zero)
 * @return the selected input type
 */
enum Core::IBoardType::InputType Naze32::getInputOnPort(int port_num)
{
    if (port_num != 0)
        return INPUT_TYPE_UNKNOWN;

    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *uavoManager = pm->getObject<UAVObjectManager>();
    HwNaze32 *hwNaze32 = HwNaze32::GetInstance(uavoManager);
    Q_ASSERT(hwNaze32);
    if (!hwNaze32)
        return INPUT_TYPE_UNKNOWN;

    HwNaze32::DataFields settings = hwNaze32->getData();

    switch(settings.RcvrPort) {
    case HwNaze32::RCVRPORT_PPM:
        return INPUT_TYPE_PPM;
    case HwNaze32::RCVRPORT_PWM:
        return INPUT_TYPE_PPM;
    case HwNaze32::RCVRPORT_SBUS:
        return INPUT_TYPE_SBUS;
    case HwNaze32::RCVRPORT_DSM:
        return INPUT_TYPE_DSM;
    default:
        return INPUT_TYPE_UNKNOWN;
    }
}

int Naze32::queryMaxGyroRate()
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *uavoManager = pm->getObject<UAVObjectManager>();
    HwNaze32 *hwNaze32 = HwNaze32::GetInstance(uavoManager);
    Q_ASSERT(hwNaze32);
    if (!hwNaze32)
        return 0;

    HwNaze32::DataFields settings = hwNaze32->getData();

    switch(settings.GyroRange) {
    case HwNaze32::GYRORANGE_250:
        return 250;
    case HwNaze32::GYRORANGE_500:
        return 500;
    case HwNaze32::GYRORANGE_1000:
        return 1000;
    case HwNaze32::GYRORANGE_2000:
        return 2000;
    default:
        return 500;
    }
}
