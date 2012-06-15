#ifndef LIDARPARSER_DATASOURCE_H
#define LIDARPARSER_DATASOURCE_H

#include "Types.h"
#include <fstream>

namespace LIDAR
{	
	typedef std::basic_istream< Byte, std::char_traits< Byte > > IBStream;
	typedef std::basic_ifstream< Byte, std::char_traits< Byte > > IBFStream;

	enum eDataSource
	{
		DataSource_IStream,
		DataSource_File,
		DataSource_MemoryArray, //, Array of data in memory
	};

	class DataSource
	{
	public:
		DataSource( IBStream& dataStream, eDataSource sourceType = DataSource_IStream ) 
			: m_dataStream(dataStream),
			m_sourceType( sourceType )
		{}
		virtual ~DataSource(){}
		
		inline IBStream& istream()
		{ return m_dataStream; }

		/** Dereference directly to IStream for code simplicity
		*/
		inline IBStream& operator *()
		{ return m_dataStream; }
		
		inline IBStream* operator ->()
		{ return &m_dataStream; }

		/** Go back to start of the data source
		*/
		void reset()
		{ m_dataStream.seekg( std::ios_base::beg ); }

		bool valid() const
		{ return m_dataStream.good(); }

	protected:
		eDataSource m_sourceType; //< Source stream type for informative/debug purposes
		IBStream& m_dataStream; //, Reference to a input stream for file, buffer, or string reading
	};

	class FileDataSource : public DataSource
	{
	public:
		FileDataSource( const Char* filename )
			: DataSource(m_fileStream, DataSource_File),
			m_fileStream(filename, std::ios::binary ) {}

	private:
		IBFStream m_fileStream;
	};
	
	/*
	class ArrayDataSource : public DataSource
	{
	private:
		struct MemoryBuffer : std::basic_streambuf< Byte, std::char_traits<Byte> >
		{
			MemoryBuffer( Byte* b, Byte* e) 
			{ this->setg(b, b, e);  }
		};

	public:
		ArrayDataSource( const Byte* dataArray, Count dataArrayByteCount  )
			: DataSource( m_memoryStream, DataSource_MemoryArray ),
			m_memoryBuffer( const_cast<Byte*>(dataArray), const_cast<Byte*>(dataArray) + dataArrayByteCount ),
			m_memoryStream( &m_memoryBuffer ) 
		{}

	private:
		MemoryBuffer m_memoryBuffer;
		IBStream m_memoryStream;
	};
	*/

} //END: LIDAR
#endif