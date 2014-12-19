/**
 ******************************************************************************
 * @file       miscplugin.cpp
<<<<<<< HEAD
 * @author     John Ihlein, Copyright (C) 2014
=======
 * @author     John Ihlein, Copyright (C) 2014.
>>>>>>> GCS: Add board plugin for miscellaneous flight control boards
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

#include "miscplugin.h"
#include "naze32pro.h"
#include <QtPlugin>


MiscPlugin::MiscPlugin()
{
   // Do nothing
}

MiscPlugin::~MiscPlugin()
{
   // Do nothing
}

bool MiscPlugin::initialize(const QStringList& args, QString *errMsg)
{
   Q_UNUSED(args);
   Q_UNUSED(errMsg);
   return true;
}

void MiscPlugin::extensionsInitialized()
{
    /**
     * Create the board objects here.
     *
     */
    Naze32Pro* naze32pro = new Naze32Pro();
    addAutoReleasedObject(naze32pro);
}

void MiscPlugin::shutdown()
{
}
