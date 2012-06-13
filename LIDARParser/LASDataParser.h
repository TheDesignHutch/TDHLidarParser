#ifndef LIDARPARSER_LASDATAPARSER_H
#define LIDARPARSER_LASDATAPARSER_H

#include "LIDARParser/Types.h"
#include "LIDARParser/DataSource.h"

namespace LIDAR
{
	namespace LAS
	{
		#pragma pack(push, 1)
		#pragma pack(1)

		// Header
		struct Header
		{											//Item										Format				Size		Required
			char signature[4];						//File Signature (�LASF�)					char[4]				4 bytes		*	
			unsigned long _pad1;					//Reserved									unsigned long		4 bytes
			unsigned long guidData1;				//GUID data 1								unsigned long		4 bytes
			unsigned short guidData2;				//GUID data 2								unsigned short		2 byte
			unsigned short guidData3;				//GUID data 3								unsigned short		2 byte
			unsigned char guidData4[8];				//GUID data 4								unsigned char[8]	8 bytes
			unsigned char majorVersion;				//Version Major								unsigned char		1 byte		*
			unsigned char minorVersion;				//Version Minor								unsigned char		1 byte		*
			char systemID[32];						//System Identifier							char[32]			32 bytes	*
			char generatingSoftware[32];			//Generating Software						char[32]			32 bytes	*
			unsigned short flightDate;				//Flight Date Julian						unsigned short		2 bytes
			unsigned short year;					//Year										unsigned short		2 bytes
			unsigned short headerSize;				//Header Size								unsigned short		2 bytes		*
			unsigned long dataOffset;				//Offset to data							unsigned long		4 bytes		*
			unsigned long numRecords;				//Number of variable length records			unsigned long		4 bytes		*
			unsigned char pointFormatID;			//Point Data Format ID (0-99 for spec)		unsigned char		1 byte		*
			unsigned short pointRecordLength;		//Point Data Record Length					unsigned short		2 bytes		*
			unsigned long numPointRecords;			//Number of point records					unsigned long		4 bytes		*
			unsigned long numPointByReturn[5];		//Number of points by return				unsigned long[5]	20 bytes	*
			double xScale;							//X scale factor							double				8 bytes		*
			double yScale;							//Y scale factor							double				8 bytes		*
			double zScale;							//Z scale factor							double				8 bytes		*
			double xOffset;							//X offset									double				8 bytes		*
			double yOffset;							//Y offset									double				8 bytes		*
			double zOffset;							//Z offset									double				8 bytes		*
			double xMax;							//Max X										double				8 bytes		*
			double xMin;							//Min X										double				8 bytes		*
			double yMax;							//Max Y										double				8 bytes		*
			double yMin;							//Min Y										double				8 bytes		*
			double zMax;							//Max Z										double				8 bytes		*
			double zMin;							//Min Z										double				8 bytes		*
		};

		/**
			LAS Point data type version 0
		*/
		struct PointV0
		{											//Item										Format			Size		Required
			long			x;						//X											long			4 bytes		*
			long			y;						//Y											long			4 bytes		*
			long			z;						//Z											long			4 bytes		*
			unsigned short	intensity;				//Intensity									unsigned short	2 bytes
			unsigned char	returnNumber : 3;		//Return Number								3 bits			3 bits		*
			unsigned char	numReturns : 3;			//Number of Returns (given pulse)			3 bits			3 bits		*
			unsigned char	scanDirectionFlag : 1;	//Scan Direction Flag						1 bit			1 bit		*
			unsigned char	edgeOfFlightLine : 1;	//Edge of Flight Line						1 bit			1 bit		*
			unsigned char	classification;			//Classification							unsigned char	1 byte
			char			scanAngle;				//Scan Angle Rank (-90 to +90) � Left side	char			1 byte		*
			unsigned char	userBitField;			//User Bit Field							unsigned short	1 byte
			unsigned short	pointSourceID;			//Point source Id							unsigned short	2 bytes		*
		};

		
		/**
			LAS Point data type version 0
			\remarks This extends PointV0 to include GPS time
		*/
		struct PointV1 : PointV0
		{
			double GPSTime; //< GPS Time double 8 bytes
		};

		struct PointV2 : PointV0
		{
			unsigned short red;
			unsigned short green;
			unsigned short blue;
		};

		struct PointV3 : PointV1
		{
			unsigned short red;
			unsigned short green;
			unsigned short blue;
		};

		
		// used to support any new records that come along
		struct VariableLengthRecordHeader
		{										// VARIABLE LENGTH RECORD HEADER
			unsigned short	recordSignature;	//Item										Format			Size		Required
			char			userID[16];			//Record Signature (0xAABB)					unsigned short	2 bytes		*
			unsigned short	recordID;			//User ID									char[16]		16 bytes	*
			unsigned short	recordLength;		//Record ID									unsigned short	2 bytes		*
			char			description[32];	//Record Length After Header				unsigned short	2 bytes		*
		};										//Description								char[32]		32 bytes

		#pragma pack(pop)

		class Parser
		{
		public:
			typedef Header Header;
			typedef PointV0 Point;

			//Bring point types into the parsers namespace
			typedef PointV0 PointV0;
			typedef PointV1 PointV1;
			typedef PointV2 PointV2;
			typedef PointV3 PointV3;
		public:		
			Parser() {}
			~Parser() {}

			const Header* getHeader( DataSource& dataSource );
			const Point* getPoint( DataSource& dataSource );

			inline UInt32 getPointTypeID() const
			{ return m_header.pointFormatID; }

			inline Size recordSize() const
			{ return m_pointSize; }	
		
		private:
			Header m_header;
			Size m_pointSize;//< Stored poitn size based on getPointTypeID()

		};
	
	} //END: LAS

	typedef LIDARParser< LAS::Parser > LASParser;

}; //END: LIDAR

#endif