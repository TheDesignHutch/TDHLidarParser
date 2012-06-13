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

		const Parser::Header* Parser::getHeader( DataSource& dataSource )
		{
			if ( !dataSource.valid() )
				return 0;

			dataSource->read( (Byte*)&m_header, sizeof(m_header) );
			assert( sizeof(m_header) == dataSource->gcount() );

			assert( getPointTypeID() < kNumPointTypes );
			m_pointSize = kPointTypeSizeLookup[getPointTypeID()];

			return &m_header;
		}

		const Parser::Point* Parser::getPoint( DataSource& dataSource )
		{
			return 0;
		}

	} //END: LAS

}; //END: LIDAR

//#include <assert.h>
//#include <malloc.h>
//#include <stdio.h>
//#include <tchar.h>
//#include "LAS.h"
//
//
//#define MAKE_4CC(a,b,c,d) ((((unsigned int)a) << 24) | (((unsigned int)b) << 16) | (((unsigned int)c) << 8) | ((unsigned int)d))
//
//
//namespace LIDAR
//{
//	template <typename PointData> unsigned int readPointsTemplate(FILE *lasFile, LIDAR::LASStreamingParser::PointV0 *&pointRecords, unsigned int maxPoints)
//	{
//		const unsigned int totalLength = sizeof(PointData) * maxPoints;
//		size_t readLength = fread(pointRecords, 1, totalLength, lasFile);
//		return (static_cast<unsigned int>(readLength) / sizeof(PointData));
//	}
//
//	static const unsigned int kTypeSizeLookup[] = {	sizeof(LASStreamingParser::PointV0),
//													sizeof(LASStreamingParser::PointV1),
//													sizeof(LASStreamingParser::PointV2),
//													sizeof(LASStreamingParser::PointV3) };
//	static const unsigned int kNumTypes = sizeof(kTypeSizeLookup) / sizeof(unsigned int);
//}; // END of namespace LIDAR
//
//
//
//
//LIDAR::LASStreamingParser::LASStreamingParser() : mHeader(0), mFile(0), mPointRecordBuffer(0),
//	mBufferOffset(0), mBufferSize(0), mNumParsedPoints(0), mNumProcessedPoints(0)
//{
//}
//
//LIDAR::LASStreamingParser::~LASStreamingParser()
//{ 
//	delete mPointRecordBuffer;
//	delete mHeader;
//	if(mFile)
//	{ fclose(mFile); }
//}
//
//bool LIDAR::LASStreamingParser::readVariableLengthRecords()
//{
//	for(unsigned int i = 0; i < mHeader->numRecords; ++i)
//	{
//		size_t readLength;
//		VarLengthRecordHeader recordHeader;
//		readLength = fread(&recordHeader, 1, sizeof(VarLengthRecordHeader), mFile);
//		assert(readLength == sizeof(VarLengthRecordHeader));
//		fseek(mFile, recordHeader.recordLength, SEEK_CUR);
//	}
//	return true;
//}
//
//bool LIDAR::LASStreamingParser::readPoints()
//{
//	unsigned int maxPoints = mBufferSize / recordSize();
//	int pointRecordLength = 0;
//	switch(mHeader->pointFormatID)
//	{
//	case 0:
//		assert(sizeof(PointV0) == mHeader->pointRecordLength);
//		mNumParsedPoints += readPointsTemplate<PointV0>(mFile, mPointRecordBuffer, maxPoints);
//		break;
//	case 1:
//		assert(sizeof(PointV1) == mHeader->pointRecordLength);
//		mNumParsedPoints += readPointsTemplate<PointV1>(mFile, mPointRecordBuffer, maxPoints);
//		break;
//	case 2:
//		assert(sizeof(PointV2) == mHeader->pointRecordLength);
//		mNumParsedPoints += readPointsTemplate<PointV2>(mFile, mPointRecordBuffer, maxPoints);
//		break;
//	case 3:
//		assert(sizeof(PointV3) == mHeader->pointRecordLength);
//		mNumParsedPoints += readPointsTemplate<PointV3>(mFile, mPointRecordBuffer, maxPoints);
//		break;
//	default:
//		printf("Unknown point format, assuming > 1\n");
//		return false;
//	}
//	mBufferOffset = 0;
//	return true;
//}
//
//bool LIDAR::LASStreamingParser::parse(const char *filename)
//{
//	delete mHeader;
//	mHeader = 0;
//	errno_t error = fopen_s(&mFile, filename, "rb");
//	if(!mFile)
//	{ return false; }
//	size_t readLength;
//	mHeader = new Header;
//	readLength = fread(mHeader, 1, sizeof(Header), mFile);
//	assert(readLength == sizeof(Header));
//	unsigned int LASFval = MAKE_4CC('L', 'A', 'S', 'F');
//	unsigned int readLASFval = MAKE_4CC(mHeader->signature[0], mHeader->signature[1], mHeader->signature[2], mHeader->signature[3]);
//	if( (readLASFval != LASFval) || (readLength != mHeader->headerSize) )
//	{ return false; }
//
//	bool result = true;
//	result &= readVariableLengthRecords();
//	fseek(mFile, mHeader->dataOffset, SEEK_SET);
//
//	mBufferOffset = mBufferSize + 1;
//	mNumParsedPoints = 0;
//	mNumProcessedPoints = 0;
//
//	return result;
//}
//
//void LIDAR::LASStreamingParser::setBufferSize(unsigned int numBytes)
//{
//	assert(mHeader == 0);
//	free(mPointRecordBuffer);
//	mBufferSize = numBytes;
//	mPointRecordBuffer = reinterpret_cast<PointV0*>(malloc(mBufferSize));
//	assert(mPointRecordBuffer);
//}
//
//unsigned int LIDAR::LASStreamingParser::pointFormatID() const
//{
//	if(!mHeader)
//	{ return kInvalidPointId; }
//	return mHeader->pointFormatID;
//}
//
//const LIDAR::LASStreamingParser::PointV0*	LIDAR::LASStreamingParser::next()
//{
//	if(!mHeader || (mNumProcessedPoints == mHeader->numPointRecords) )
//	{ return 0; }
//	if( (mBufferOffset + recordSize()) >= mBufferSize)
//	{ readPoints(); }
//	unsigned int currentOffset = mBufferOffset;
//	mBufferOffset += recordSize();
//	++mNumProcessedPoints;
//	const unsigned char *buf = reinterpret_cast<const unsigned char*>(mPointRecordBuffer);
//	buf += currentOffset;
//	return reinterpret_cast<const PointV0*>(buf);
//}
//
//void LIDAR::LASStreamingParser::restart()
//{
//	fseek(mFile, mHeader->dataOffset, SEEK_SET);
//
//	mBufferOffset = mBufferSize + 1;
//	mNumParsedPoints = 0;
//	mNumProcessedPoints = 0;
//}
//
//const LIDAR::LASStreamingParser::Header*	LIDAR::LASStreamingParser::header() const
//{ return mHeader; }
//
//unsigned int LIDAR::LASStreamingParser::recordSize() const
//{
//	if(!mHeader || (mHeader->pointFormatID >= kNumTypes) )
//	{ return 0; }
//	return kTypeSizeLookup[mHeader->pointFormatID];
//}



//
//	bool PropertiesXMLConfigurator::loadFromFile( const Char* filename )
//	{
//		if ( !fileExists(filename) )
//		{
//			_Log << Log::Message << "Properties not loaded as '" << filename << "' does nto exist." << Log::Flush;
//			return false;
//		}
//
//		IFStream ifstream( filename );
//		if ( !ifstream.is_open() )
//		{
//			_Log << Log::Error << "Failed to load properties configuration from '" << filename << "'. Make sure the file is not lcoked and you have permission to access the file." << Log::Flush;
//			return false;
//		}
//
//		return loadFromStream( ifstream );
//	}
//
//
//	bool PropertiesXMLConfigurator::loadFromText( const Char* text, Count length )
//	{
//		assert(text);
//#if 0
//		std::istrstream istream( text, length ? length : strlen(text) );
//#else
//		struct MemoryBuffer : std::basic_streambuf< Char, std::char_traits< Char> >
//		{
//			MemoryBuffer( Char* b, Char* e) 
//			{ this->setg(b, b, e);  }
//		};
//		//TODO: must cast away const to pass this as a IStream! Cannot change IStream to use const Char as IFStream can then no longer be passed as an IStream and the IFStream cannot compile with a const data type!
//		MemoryBuffer textBuffer( const_cast<Char*>(text), const_cast<Char*>(text + (length ? length : strlen(text))) );
//		IStream istream( &textBuffer );
//#endif
//		return loadFromStream( istream );
//	}
//	
//	bool PropertiesXMLConfigurator::saveToFile( const Char* filename )
//	{
//		OFStream ofstream( filename );
//		if ( !ofstream.is_open() )
//		{
//			assert(false);
//			_Log << Log::Error << "Failed to save properties configuration to '" << filename << "'. Make sure the file is not lcoked and you have permission to access the file." << Log::Flush;
//			return false;
//		}
//
//		return saveToStream( ofstream );
//	}
//
//	bool PropertiesXMLConfigurator::setPropertyValue( const String& strname, const String& strvalue )
//	{
//		//First we need to find the property by name
//		NameIndexLookup::const_iterator iPropIndex = m_nameIndexLookup.find( strname );
//		if ( iPropIndex == m_nameIndexLookup.end() )
//		{
//			_Log << Log::RecoverableError << "Failed to find property '" << strname << "'. This name may have been changed or removed, check your configuration." << Log::Flush;
//			//assert(false);
//			return false;
//		}
//
//		//Now to get the property reference
//		Index iProp = iPropIndex->second;
//		Property prop = m_properties->getPropertyByIndex( iProp );
//		if ( !prop.isValid() ) //, SANITY Check! should not ever happen as we jsut built the list fomr the properties in the container moments ago!
//		{
//			_Log << Log::Error << "Failed to access/set property '" << strname << "'." << Log::Flush;
//			assert(false);
//			return false;
//		}
//
//		Variant valueVariant = Variant(strvalue).toVariant( prop.value().type() );
//		assert( valueVariant.type() != Variant::Type_Null );
//		prop.value() = valueVariant;
//		return true;
//	}
//
//	static const String kPropertyTag = _T("property");
//	static const String kNameTag = _T("name");
//
//	bool PropertiesXMLConfigurator::loadFromStream( IStream& stream )
//	{
//		IO::XMLStreamReader xmlStream( stream );
//
//		const Char* value;
//		while ( value = xmlStream.getElement() )
//		{
//			if ( kPropertyTag == value )
//			{
//				String name;
//				while ( value = xmlStream.getAttribute() )
//				{
//					if ( kNameTag == value )
//						name = xmlStream.getAttributeValue();
//					else
//					{
//						logWarning( "Unsupported XMl attribute '%s' found in properties configuration", value );
//						assert(false);
//					}
//				}
//				assert( name.length() );
//				setPropertyValue( name, xmlStream.getElementValue() );
//				
//				//terminate this tag!
//				const Char* childElement = xmlStream.getElement();
//				assert(childElement == 0);	
//			}
//			else
//			{
//				logWarning( "Unsupported XMl element '%s' found in properties configuration", value );
//				assert(false);
//			}
//		}
//		return true;
//	}
//
//	bool PropertiesXMLConfigurator::saveToStream( OStream& stream)
//	{
//		for ( Index iProp = 0; iProp != m_properties->getPropertyCount(); ++iProp )
//		{
//			Property prop = m_properties->getPropertyByIndex( iProp );
//			stream << '<' << kPropertyTag << ' ' << kNameTag << "=\"" << m_indexNameLookup[iProp] << "\">" 
//					<< prop.value().toString()
//					<< "</" << kPropertyTag << "> <!-- " << prop.description() << " -->\n";
//		}
//
//		return true;
//	}