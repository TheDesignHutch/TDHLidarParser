#include "StdAfx.h"

#include "LIDARParser.h"
#include "LASDataParser.h"

namespace LIDAR
{
	namespace LAS
	{

		static const Size kPointTypeSizeLookup[] = 
		{	
			sizeof(PointV0),
			sizeof(PointV1),
			sizeof(PointV2),
			sizeof(PointV3) 
		};
		static const Count kNumPointTypes = sizeof(kPointTypeSizeLookup) / sizeof(*kPointTypeSizeLookup);

		bool Parser::readHeader( DataSource& dataSource )
		{
			if ( !dataSource.valid() )
				return assert(false), false; //< TODO: error reporting

			dataSource->read( (Byte*)&m_header, sizeof(m_header) );
			if( sizeof(m_header) != dataSource->gcount() )
				return assert(false), false; //< TODO: error reporting

			if ( memcmp( m_header.signature, "LASF", 4 ) != 0 )
				return assert(false), false; //< TODO: error reporting

			assert( m_header.pointFormatID < kNumPointTypes );
			assert( m_header.pointRecordLength == kPointTypeSizeLookup[m_header.pointFormatID] );

			
			for(unsigned int i = 0; i < m_header.numRecords; ++i)
			{
				VariableLengthRecordHeader recordHeader;
				dataSource->read( (Byte*)&recordHeader, sizeof(recordHeader) );
				assert( dataSource->gcount() == sizeof(recordHeader));
				dataSource->ignore( recordHeader.recordLength );//, std::ios_base::cur );
			}

			size_t pos1 = dataSource->tellg();
			dataSource->seekg( 0, std::ios_base::end );
			size_t pos = dataSource->tellg();
			dataSource->seekg( m_header.dataOffset );

			pos = dataSource->tellg();
			return true;
		}

		bool Parser::readPoint( DataSource& dataSource, Point* point, Size pointSize )
		{
			Size readLength = min( pointSize, (Size)m_header.pointRecordLength );
			//if( m_header.pointRecordLength > pointSize )
			//	return assert(false), false; //< TODO: error reporting
			
			dataSource->read( (Byte*)point, readLength );			
			if ( readLength < m_header.pointRecordLength )
				dataSource->ignore( m_header.pointRecordLength-readLength ); //, Ignore the rest of this points data!

			return dataSource.valid();
		}

		Count Parser::readPoints( DataSource& dataSource, Byte* pointBuffer, Count pointBufferCount, Count pointSize )
		{
			assert( m_header.pointRecordLength == pointSize );

			assert( !dataSource->fail() );
			assert( !dataSource->eof() );

			//size_t pos1 = dataSource->tellg();
			Size readByteTotal = pointSize*pointBufferCount;
			dataSource->read( (Byte*)pointBuffer,  readByteTotal/*readByteTotal*/ );			

			//size_t pos = dataSource->tellg();

			assert( !dataSource->eof() );
			assert( !dataSource->fail() );
			assert( dataSource->good() );

			if ( dataSource->gcount() == readByteTotal )
				return pointBufferCount;

			assert( (dataSource->gcount() % pointSize) == 0 );
			return dataSource->gcount() / pointSize;
		}

	} //END: LAS

	template class HeaderReader<LAS::Parser>;

}; //END: LIDAR
