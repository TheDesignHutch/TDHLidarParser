#ifndef LIDARPARSER_H
#define LIDARPARSER_H

#include "LIDARParser/Types.h"

namespace LIDAR
{	
	//forward declarations
	class DataSource;

	class DataParser
	{
	public:
		typedef void Header;
		typedef void Point;
	public:
		Header* getHeader( DataSource& dataSource );
		Point* getPoint( DataSource& dataSource );
		UInt32 getPointTypeID() const
		{ return 0; }
	};

	template < class _DataParser >
	class HeaderReader : public _DataParser::Header
	{
	public:
		inline UInt32 getPointTypeID() const;
		inline UInt32 getPointCount() const;
		inline UInt32 getPointSize() const;
	};

	template< Count _Capacity >
	class PointBuffer
	{
	public:

		/** Generic point iterator
			\remarks Use IteratorT for faster type-specific iteration
		*/
		class Iterator
		{
		public:
			inline Iterator( Byte* data, Size pointSize ) 
				: m_pointSize(pointSize),
				m_pointData(data)
			{}

			template< class _Point >
			operator _Point*()
			{ assert( sizeof(_Point) <= m_pointSize); return (_Point*)(m_pointData); }

			template< class _Point >
			_Point* as()
			{ assert( sizeof(_Point) <= m_pointSize); return (_Point*)(m_pointData); }

			inline Iterator& operator ++()
			{ m_pointData += m_pointSize; return *this; }

			inline Iterator operator +( Count count )
			{ return Iterator(m_pointData+count*m_pointSize, m_pointSize); }

			inline bool operator != ( const Iterator& rhs ) const
			{ return m_pointData != rhs.m_pointData; }

			
			Size pointSize() const
			{ return m_pointSize; }
			
			Byte* pointData() const
			{ return m_pointData; }
		private:
			Size m_pointSize;
			Byte* m_pointData;
		};

		/** Type specific iterator
		*/
		template< class _Point >
		class IteratorT
		{
		public:
			inline IteratorT( Byte* data, Size pointSize ) 
				: m_pointData(data)
			{ assert( sizeof(_Point) == pointSize ); }

			inline IteratorT( const Iterator& it ) 
				: m_pointData( (_Point*)it.pointData() )
			{ assert( sizeof(_Point) == it.pointSize() ); }

			operator _Point*()
			{ return m_pointData; }

			inline IteratorT& operator ++()
			{ ++m_pointData; return *this; }

			inline IteratorT operator +( Count count )
			{ return IteratorT(m_pointData+count, sizeof(_Point) ); }

			inline bool operator != ( const IteratorT& rhs ) const
			{ return m_pointData != rhs.m_pointData; }
		private:
			_Point* m_pointData;
		};

	public:
		PointBuffer( Size pointSize )
			: m_pointSize(pointSize),
			m_pointData( new Byte[pointSize*_Capacity] )
		{ assert(m_pointData); assert(pointSize); }

		~PointBuffer()
		{ delete[] m_pointData; }

		template< class _Point >
		_Point* operator[] ( Index i )
		{ assert( i < _Capacity); return (_Point*)(m_pointData + i*m_pointSize); }

		inline Count capacity() const
		{ return _Capacity; }

		inline Size size() const
		{ return _Capacity*m_pointSize; }

		inline Size pointSize() const
		{ return m_pointSize; }

		inline Byte* c_ptr()
		{ return m_pointData; }

		inline const Byte* c_ptr() const
		{ return m_pointData; }

		inline Iterator begin()
		{ return Iterator(m_pointData,m_pointSize); }

		
	protected:
		Byte* m_pointData;	
		Size m_pointSize;
	};

	template < class _DataParser >
	class LIDARParser
	{
	public:
		typedef _DataParser DataParser;
		typedef typename HeaderReader<_DataParser> Header;
		//typedef typename _DataParser::Header HeaderData;
		typedef typename _DataParser::Point Point;

	public:		

		LIDARParser( DataSource& dataSource ) 
			: m_dataSource(dataSource)
		{}

		~LIDARParser()
		{}

		bool readHeader()
		{ 
			m_dataSource.reset();
			bool ret = m_dataParser.readHeader(m_dataSource); 
			m_pointsRemaining = getPointCount();
			return ret;
		}

		inline const Header& getHeader() const
		{
			//TODO: potentially unsafe? Return the header with a templated accessor interface via casting for speed/convenience/flexibility!
			return (const Header&)( m_dataParser.getHeader() );
		}

		template< class _Point >
		inline bool readPoint( _Point* point/*, Size pointSize*/ )
		{ 
			assert(m_pointsRemaining);
			bool ret = m_dataParser.readPoint(m_dataSource, point, sizeof(*pointSize) );
			if ( ret ) --m_pointsRemaining;
			return ret;
		}

		template < Count _Capacity >
		inline Count readPoints( PointBuffer<_Capacity>& buffer )
		{ 
			Count ret = m_dataParser.readPoints( m_dataSource, buffer.c_ptr(), min(buffer.capacity(),m_pointsRemaining), buffer.pointSize() );
			m_pointsRemaining -= ret;
			return ret;
		}

		//Count readPoints( const Byte* pointBuffer, Size pointBufferSizeBytes )
		//{ return m_dataParser.readPoints( m_dataSource, pointBuffer, pointBufferSizeBytes ); }
			
		inline UInt32 getPointTypeID() const
		{ return getHeader().getPointTypeID(); }
		
		inline UInt32 getPointCount() const
		{ return getHeader().getPointCount(); }

		inline UInt32 getPointSize() const
		{ return getHeader().getPointSize(); }

	private:
		DataSource& m_dataSource;
		DataParser m_dataParser;
		Count m_pointsRemaining; //< NUmber fo points remaining in file
	};

} //END: LIDAR

#endif