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
	LIDAR::FileDataSource lasSource( _T("../../LIDARData/LAS/TestData1.las") );
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

	//Get the point offset wihtin the unit system of the lidar point data bounds
	//Using this we offset all data to 0,0,0 origin for this file so can store position as Uint32
	// - This is to avoid use of double precision during loading in the first few stages
	LIDAR::Int32 offsetX = LIDAR::Int32(lasHeader.xMin/lasHeader.xScale);
	LIDAR::Int32 offsetY = LIDAR::Int32(lasHeader.yMin/lasHeader.yScale);
	LIDAR::Int32 offsetZ = LIDAR::Int32(lasHeader.zMin/lasHeader.zScale);

	//Get the 'extent' or 'bounds' of the data which we need to calculate the scale factor below
	LIDAR::UInt32 extentX = LIDAR::Int32((lasHeader.xMax-lasHeader.xMin)/lasHeader.xScale);
	LIDAR::UInt32 extentY = LIDAR::Int32((lasHeader.yMax-lasHeader.yMin)/lasHeader.yScale);
	LIDAR::UInt32 extentZ = LIDAR::Int32((lasHeader.zMax-lasHeader.zMin)/lasHeader.zScale);

	//We sacle point values to the full UInt32 range to maintain the highest level 
	// of accuracy and allow fast oct-tree generation while only using integer operations and
	// avoid double maths or floating point errors
	LIDAR::UInt32 scaleX = UINT_MAX/extentX;
	LIDAR::UInt32 scaleY = UINT_MAX/extentY;
	LIDAR::UInt32 scaleZ = UINT_MAX/extentZ;

	//In this test we check the min-max boudns of the data are wihtin expected range
	LIDAR::UInt32 minX = UINT_MAX, minY = UINT_MAX, minZ = UINT_MAX;
	LIDAR::UInt32 maxX = 0, maxY = 0, maxZ = 0;

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

			LIDAR::UInt32 x = LIDAR::UInt32(point->x - offsetX) * scaleX;
			LIDAR::UInt32 y = LIDAR::UInt32(point->y - offsetY) * scaleY;
			LIDAR::UInt32 z = LIDAR::UInt32(point->z - offsetZ) * scaleZ;

			minX = LIDAR::min(x, minX);
			maxX = LIDAR::max(x, maxX);

			minY = LIDAR::min(y, minY);
			maxY = LIDAR::max(y, maxY);

			minZ = LIDAR::min(z, minZ);
			maxZ = LIDAR::max(z, maxZ);
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
	printf("Header\n\tscale: %Lf %Lf %Lf \n\toffset: %Lf %Lf %Lf\n",
		lasHeader.xScale, lasHeader.yScale, lasHeader.zScale, 
		lasHeader.xOffset, lasHeader.yOffset, lasHeader.zOffset );
	printf("Header\n\tmin: %Lf %Lf %Lf \n\tmax: %Lf %Lf %Lf \n\textent: %Lf %Lf %Lf\n", 
		lasHeader.xMin, lasHeader.yMin, lasHeader.zMin, 
		lasHeader.xMax, lasHeader.yMax, lasHeader.zMax, 
		lasHeader.xMax-lasHeader.xMin, lasHeader.yMax-lasHeader.yMin, lasHeader.zMax-lasHeader.zMin );
	printf("Data (limit %u) \n\tmin: %u %u %u \n\tmax: %u %u %u \n\textent: %u %u %u\n", 
		UINT_MAX,
		minX, minY, minZ, 
		maxX, maxY, maxZ, 
		maxX-minX, maxY-minY, maxZ-minZ );

	printf( "Elapsed for %u points: %u ms\n", kPointCount, end-start );

	assert( remainingCount == 0);

	return 0;
}

