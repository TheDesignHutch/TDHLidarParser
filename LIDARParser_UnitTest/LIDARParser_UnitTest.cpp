// LIDARParser_UnitTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "LIDARParser/DataSource.h"
#include "LIDARParser/LIDARParser.h"
#include "LIDARParser/LASDataParser.h"


int _tmain(int argc, _TCHAR* argv[])
{
	LIDAR::FileDataSource lasSource( _T("./Data/srs.las") );
	if ( !lasSource.valid() )
		return assert(false), -1;

	LIDAR::LASParser lasParser( lasSource );

	const LIDAR::LASParser::Header* lasHeader = lasParser.begin();


	return 0;
}

