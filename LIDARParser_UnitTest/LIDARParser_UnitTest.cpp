// LIDARParser_UnitTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "LIDARParser/DataSource.h"
#include "LIDARParser/LIDARParser.h"
#include "LIDARParser/LASDataParser.h"


int _tmain(int argc, _TCHAR* argv[])
{
	//We are going to use the LAS parser
	typedef LIDAR::LASParser LIDARParser;

	//Open the LIDAR source from file
	// - ALternatively from a memory array or std stream
	LIDAR::FileDataSource lasSource( _T("./Data/TestData1 - Copy.las") );
	if ( !lasSource.valid() )
		return assert(false), -1;

	LIDARParser lasParser( lasSource );
	if ( !lasParser.readHeader() )
		return assert(false), -1;

	const LIDARParser::Header& lasHeader = lasParser.getHeader();	
	LIDAR::Count kPointCount = lasHeader.getPointCount();
	LIDAR::Count remainingCount = kPointCount;
	LIDAR::Count pointSize = lasHeader.getPointSize();
	LIDAR::Count pointTypeID = lasHeader.getPointTypeID();


	
		LIDAR::UInt32 totalX = 0;
	DWORD start = timeGetTime();

#if 1
	typedef LIDAR::PointBuffer<4096> PointBuffer; //< 1024-4096 seem to be sweet spots!
	PointBuffer buffer( lasParser.getPointSize() );
	LIDAR::Count lastRead;
	while ( lastRead = lasParser.readPoints( buffer ) )
	{
		assert( lastRead <= remainingCount );
		assert( lastRead <= buffer.capacity() );
		remainingCount -= lastRead; //< Keep a tally of number of points read!

		//Iterate over the points in the buffer
		for ( PointBuffer::Iterator iPoint = buffer.begin(), end = buffer.begin() + lastRead; iPoint != end; ++iPoint )
		{
			LIDARParser::Point* point = (LIDARParser::Point*)iPoint;
			//printf( "%i %i %i\n", point->x, point->y, point->z );

			totalX += point->x;
			totalX += point->y;
			totalX += point->z;
		}
	}
#else
	//LIDARParser::DataParser::PointV3 point;
	LIDARParser::Point point;
	while ( remainingCount && lasParser.readPoint( &point, lasParser.getPointSize()/*sizeof(point)*/ ) )
	{
			totalX += point.x;
			totalX += point.y;
			totalX += point.z;
		--remainingCount;
	}
#endif

	DWORD end = timeGetTime();
		printf("%u\n", totalX );
	printf( "Elapsed for %u points: %u ms\n", kPointCount, end-start );

	assert( remainingCount == 0);

	return 0;
}

