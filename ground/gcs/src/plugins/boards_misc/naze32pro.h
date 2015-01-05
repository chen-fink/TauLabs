/**
 ******************************************************************************
 * @file       naze32pro.h
 * @author     John Ihlein, Copyright (C) 2014
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_MiscPlugin Misc boards support Plugin
 * @{
 * @brief Plugin to support miscellaneous flight control boards
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
#ifndef NAZE32PRO_H
#define NAZE32PRO_H

#include <coreplugin/iboardtype.h>

class IBoardType;

class Naze32Pro : public Core::IBoardType
{
public:
    Naze32Pro();
    virtual ~Naze32Pro();

    virtual QString shortName();
    virtual QString boardDescription();
    virtual bool queryCapabilities(BoardCapabilities capability);
    virtual QStringList getSupportedProtocols();
    virtual QPixmap getBoardPicture();
    virtual QString getHwUAVO();

    //! Determine if this board supports configuring the receiver
    virtual bool isInputConfigurationSupported();

    /**
     * Configure the board to use an receiver input type on a port number
     * @param type the type of receiver to use
     * @param port_num which input port to configure (board specific numbering)
     */
    virtual bool setInputOnPort(enum InputType type, int port_num = 0);

    /**
     * @brief getInputOnPort get the current input type
     * @param port_num which input port to query (board specific numbering)
     * @return the currently selected input type
     */
    virtual enum InputType getInputOnPort(int port_num = 0);

    virtual int queryMaxGyroRate();

};


#endif // NAZE32PRO_H