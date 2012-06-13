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
	class LIDARParser
	{
	public:
		typedef _DataParser DataParser;
		typedef typename _DataParser::Header Header;
		typedef typename _DataParser::Point Point;

	public:		

		LIDARParser( DataSource& dataSource ) 
			: m_dataSource(dataSource)
		{}

		~LIDARParser()
		{}

		const Header* begin()
		{ 
			m_dataSource.reset();
			return m_dataParser.getHeader(m_dataSource); 
		}

		const Point* getPoint()
		{ return m_dataParser.getHeadePoint(m_dataSource); }

		UInt32 getPointTypeID() const
		{ return m_dataParser.getPointTypeID(); }

		/*
		template <> const typename DataParser::Point0* getPoint<0>() { return (const typename DataParser::Point0*)getPoint(); }
		template <> const typename DataParser::Point1* getPoint<1>() { return (const typename DataParser::Point1*)getPoint(); }
		template <> const typename DataParser::Point2* getPoint<2>() { return (const typename DataParser::Point2*)getPoint(); }
		template <> const typename DataParser::Point3* getPoint<3>() { return (const typename DataParser::Point3*)getPoint(); }
		template <> const typename DataParser::Point4* getPoint<4>() { return (const typename DataParser::Point4*)getPoint(); }
		*/
	private:
		DataSource& m_dataSource;
		DataParser m_dataParser;
	};

} //END: LIDAR

#endif