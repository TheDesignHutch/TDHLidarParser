// LIDARParser_UnitTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "LIDARParser/DataSource.h"
#include "LIDARParser/LIDARParser.h"
#include "LIDARParser/LASDataParser.h"
#include <cmath>


#define LIDAR_UINT32_TEST 1 //, Test UInt point position bounds which should be [0->UInt32_Max,0->UInt32_Max,0->UInt32_Max]
#define LIDAR_UINT32_TO_DOUBLE_TEST 1 //< Test double precision bounds which should match header
#define LIDAR_UINT32_TO_DOUBLE_ACCURACY 1 //< Test of max LIDAR_UINT32_TO_DOUBLE_TEST innacuracy

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
	const LIDAR::Count kPointCount = lasHeader.getPointCount();
	const LIDAR::Count pointSize = lasHeader.getPointSize();
	const LIDAR::Count pointTypeID = lasHeader.getPointTypeID();

	//Get the point offset wihtin the unit system of the lidar point data bounds
	//Using this we offset all data to 0,0,0 origin for this file so can store position as Uint32
	// - This is to avoid use of double precision during loading in the first few stages
	const LIDAR::Int32 offsetX = LIDAR::Int32(lasHeader.xMin/lasHeader.xScale);
	const LIDAR::Int32 offsetY = LIDAR::Int32(lasHeader.yMin/lasHeader.yScale);
	const LIDAR::Int32 offsetZ = LIDAR::Int32(lasHeader.zMin/lasHeader.zScale);

	const LIDAR::Float64 dextentX = lasHeader.xMax-lasHeader.xMin;
	const LIDAR::Float64 dextentY = lasHeader.yMax-lasHeader.yMin;
	const LIDAR::Float64 dextentZ = lasHeader.zMax-lasHeader.zMin;

	//Get the 'extent' or 'bounds' of the data which we need to calculate the scale factor below
	const LIDAR::UInt32 extentX = LIDAR::Int32(dextentX/lasHeader.xScale);
	const LIDAR::UInt32 extentY = LIDAR::Int32(dextentY/lasHeader.yScale);
	const LIDAR::UInt32 extentZ = LIDAR::Int32(dextentZ/lasHeader.zScale);

	//We sacle point values to the full UInt32 range to maintain the highest level 
	// of accuracy and allow fast oct-tree generation while only using integer operations and
	// avoid double maths or floating point errors
	const LIDAR::UInt32 scaleX = UINT_MAX/extentX;
	const LIDAR::UInt32 scaleY = UINT_MAX/extentY;
	const LIDAR::UInt32 scaleZ = UINT_MAX/extentZ;
								 

	//In this test we check the min-max boudns of the data are wihtin expected range
	LIDAR::UInt32 minX = UINT_MAX, minY = UINT_MAX, minZ = UINT_MAX;
	LIDAR::UInt32 maxX = 0, maxY = 0, maxZ = 0;

#if LIDAR_UINT32_TO_DOUBLE_TEST //< TO double test.. TODO: needs thinking through some adapter type to obtain faster and wihtout the chaf of DIY!
	const LIDAR::Float64 dextentScaleX = dextentX /UINT_MAX;
	const LIDAR::Float64 dextentScaleY = dextentY /UINT_MAX;
	const LIDAR::Float64 dextentScaleZ = dextentZ /UINT_MAX;
	
	LIDAR::Float64 dminX = DBL_MAX, dminY = DBL_MAX, dminZ = DBL_MAX;
	LIDAR::Float64 dmaxX = -DBL_MAX, dmaxY = -DBL_MAX, dmaxZ = -DBL_MAX;

#if LIDAR_UINT32_TO_DOUBLE_ACCURACY
	LIDAR::Float64 dErrmaxX = 0, dErrmaxY = 0, dErrmaxZ = 0;
#endif
#endif		
	DWORD start = timeGetTime();

	LIDAR::Count remainingPointCount = kPointCount;

	//We can read each point sequentirally but most drives have an optimal buffer/chunk size which will imporve throughput
	// NOTE: once loaded the file data will often be cached by the OS so timings may be false. Always restart your 
	// machine or reset the OS cache when making timings of this sort
#if 1
	typedef LIDAR::PointBuffer<4096> PointBuffer; //< 1024-4096 seem to be sweet spots!
	PointBuffer buffer( lasParser.getPointSize() );
	LIDAR::Count lastReadPointCount;
	while ( lastReadPointCount = lasParser.readPoints( buffer ) )
	{
		assert( lastReadPointCount <= remainingPointCount );
		assert( lastReadPointCount <= buffer.capacity() );
		remainingPointCount -= lastReadPointCount; //< Keep a tally of number of points read!

		//Iterate over the points in the buffer
		for ( PointBuffer::Iterator iPoint = buffer.begin(), end = buffer.begin() + lastReadPointCount; iPoint != end; ++iPoint )
		{
			//Point defined as LAS v0. We get type specific (via pointTypeID) in a seperate loop (TODO: Beneficial!?)
			LIDARParser::Point* point = (LIDARParser::Point*)iPoint;

			//NOTE: This code reads the point position as a [0->UInt32_Max,0->UInt32_Max,0->UInt32_Max] position
			// wihtin the LIDAR data bounds. We can preferrably store in this format to use half the space a double would.
			// - We also have direct OctTree psoitions by using the bits on 0->UInt32_Max therefore removing almost all build time
			LIDAR::UInt32 x = LIDAR::UInt32(point->x - offsetX) * scaleX;
			LIDAR::UInt32 y = LIDAR::UInt32(point->y - offsetY) * scaleY;
			LIDAR::UInt32 z = LIDAR::UInt32(point->z - offsetZ) * scaleZ;
			
#if LIDAR_UINT32_TEST
			//UInt32 range validation/test
			minX = LIDAR::min(x, minX);
			maxX = LIDAR::max(x, maxX);

			minY = LIDAR::min(y, minY);
			maxY = LIDAR::max(y, maxY);

			minZ = LIDAR::min(z, minZ);
			maxZ = LIDAR::max(z, maxZ);
#endif

#if LIDAR_UINT32_TO_DOUBLE_TEST
			//TODO: Not necessarily optimal at present. Currently intended that you woudl store a UInt32 for fast OctTree build etc.
			LIDAR::Float64 dx = lasHeader.xMin + x * dextentScaleX;
			LIDAR::Float64 dy = lasHeader.yMin + y * dextentScaleY;
			LIDAR::Float64 dz = lasHeader.zMin + z * dextentScaleZ;
			assert( dx >= lasHeader.xMin && dx <= lasHeader.xMax );
			assert( dy >= lasHeader.yMin && dy <= lasHeader.yMax );
			assert( dz >= lasHeader.zMin && dz <= lasHeader.zMax );
			
			dminX = LIDAR::min(dx, dminX);
			dmaxX = LIDAR::max(dx, dmaxX);
								   
			dminY = LIDAR::min(dy, dminY);
			dmaxY = LIDAR::max(dy, dmaxY);
								   
			dminZ = LIDAR::min(dz, dminZ);
			dmaxZ = LIDAR::max(dz, dmaxZ);
	
#if LIDAR_UINT32_TO_DOUBLE_ACCURACY //< Accuracy check
			LIDAR::Float64 tdx = point->x * lasHeader.xScale;
			LIDAR::Float64 tdy = point->y * lasHeader.yScale;
			LIDAR::Float64 tdz = point->z * lasHeader.zScale;

			LIDAR::Float64 dxDiff = abs(tdx-dx);
			LIDAR::Float64 dyDiff = abs(tdy-dy);
			LIDAR::Float64 dzDiff = abs(tdz-dz);
			dErrmaxX = LIDAR::max(dxDiff,dErrmaxX);
			dErrmaxY = LIDAR::max(dyDiff,dErrmaxY);
			dErrmaxZ = LIDAR::max(dzDiff,dErrmaxZ);

#endif


#endif


		}
		
		//Perform type specific point retrieval
		//NOTE: it is likely faster to perform the type switch at the top level instea dof embeded in the loop
		// We can also use type sepcific iterators which use inbuilt type sizing to incrmeent/offset the iterators
		switch(pointTypeID)
		{
		case LIDARParser::DataParser::PointV1::TypeID:
			{
				//Point defined as LAS v1 so use a v1 iterator
				typedef LIDARParser::DataParser::PointV1 Point;
				for ( PointBuffer::IteratorT<Point> iPoint = buffer.begin(), end = buffer.begin() + lastReadPointCount; iPoint != end; ++iPoint )
				{
					LIDARParser::DataParser::PointV1* point = iPoint;
					double GPSTime = point->GPSTime;
				}
			}
			break;
		case LIDARParser::DataParser::PointV2::TypeID:
			{
				//Point defined as LAS v2 so use a v2 iterator
				typedef LIDARParser::DataParser::PointV2 Point;
				for ( PointBuffer::IteratorT<Point> iPoint = buffer.begin(), end = buffer.begin() + lastReadPointCount; iPoint != end; ++iPoint )
				{
					LIDARParser::DataParser::PointV2* point = iPoint;
					unsigned short red = point->red;
					unsigned short green = point->green;
					unsigned short blue = point->blue;
				}
			}
			break;
		case LIDARParser::DataParser::PointV3::TypeID:
			{
				//Point defined as LAS v3 so use a v3 iterator
				typedef LIDARParser::DataParser::PointV3 Point;
				for ( PointBuffer::IteratorT<Point> iPoint = buffer.begin(), end = buffer.begin() + lastReadPointCount; iPoint != end; ++iPoint )
				{
					LIDARParser::DataParser::PointV3* point = iPoint;
					double GPSTime = point->GPSTime;
					unsigned short red = point->red;
					unsigned short green = point->green;
					unsigned short blue = point->blue;
				}
			}
			break;
		}
	}
#else
	//LIDARParser::DataParser::PointV3 point;
	LIDARParser::Point point;
	while ( remainingPointCount && lasParser.readPoint( &point, lasParser.getPointSize()/*sizeof(point)*/ ) )
	{
			totalX += point.x;
			totalX += point.y;
			totalX += point.z;
		--remainingPointCount;
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

#if LIDAR_UINT32_TEST
	printf("Data (limit %u) \n\tmin: %u %u %u \n\tmax: %u %u %u \n\textent: %u %u %u\n", 
		UINT_MAX,
		minX, minY, minZ, 
		maxX, maxY, maxZ, 
		maxX-minX, maxY-minY, maxZ-minZ );
#endif

#if LIDAR_UINT32_TO_DOUBLE_TEST
	printf("Data (FLoat64) \n\tmin: %f %f %f \n\tmax: %f %f %f \n\textent: %f %f %f\n",
		dminX, dminY, dminZ, 
		dmaxX, dmaxY, dmaxZ, 
		dmaxX-dminX, dmaxY-dminY, dmaxZ-dminZ );
#if LIDAR_UINT32_TO_DOUBLE_ACCURACY
	printf(" - Max Error \n\t %f %f %f\n",
		dErrmaxX, dErrmaxY, dErrmaxZ );
#endif
#endif

	printf( "Elapsed for %u points: %u ms\n", kPointCount, end-start );

	assert( remainingPointCount == 0);

	return 0;
}

