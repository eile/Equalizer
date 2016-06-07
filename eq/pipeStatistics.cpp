
/* Copyright (c) 2009-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2010, Daniel Nachbaur <danielnachbaur@gmail.com>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "pipeStatistics.h"

#include "config.h"
#include "pipe.h"
#include "global.h"

#include <cstdio>

#ifdef _MSC_VER
#  define snprintf _snprintf
#endif

namespace eq
{

PipeStatistics::PipeStatistics( const Statistic::Type type, Pipe* pipe )
        : StatisticSampler< Pipe >( type, pipe )
{
    const std::string& name = pipe->getName();
    if( name.empty( ))
        snprintf( event.statistic.resourceName, 32, "Pipe %s",
                  pipe->getID().getShortString().c_str( ));
    else
        snprintf( event.statistic.resourceName, 32, "%s", name.c_str( ));

    event.statistic.resourceName[31] = 0;
    event.statistic.startTime  = pipe->getConfig()->getTime();
}


PipeStatistics::~PipeStatistics()
{
    if( event.statistic.frameNumber == 0 ) // does not belong to a frame
        return;

    Config* config = _owner->getConfig();
    if( event.statistic.endTime == 0 )
        event.statistic.endTime = config->getTime();
    if( event.statistic.endTime <= event.statistic.startTime )
        event.statistic.endTime = event.statistic.startTime + 1;

    _owner->processEvent( event );
}

}
