/*
 *	This file is part of TDHLidarParser - A fast, efficient, LIDAR file/stream parsing library.
 *	
 *	Copyright (C) 2012-2013 Craig Hutchinson <contact@theDeignHutch.com>
 *	
 *	TDHLidarParser is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *	
 *	TDHLidarParser is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *	
 *	You should have received a copy of the GNU General Public License
 *	along with TDHLidarParser.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef LIDARPARSER_TYPES_H
#define LIDARPARSER_TYPES_H

#define NOMINMAX
#include <TCHAR.h>
#include <cfloat>

#define USE_TCHAR //< Set this to enable project specific unicode/utf8 build
#ifdef USE_TCHAR
#	define RTCHAR TChar
#else
#	define RTCHAR Char8
#endif

namespace LIDAR
{		
	
	typedef float Float32; 
	typedef double Float64; //Long Float32 = double

#if REAL_DOUBLE
	typedef Float64 Real; //Real
#else
	typedef Float32 Real; //Real
#endif

	typedef char Char8;
	typedef wchar_t Char16;
	typedef TCHAR TChar;
	typedef RTCHAR Char; //< Specified via USE_TCHAR precompiler to be either Char* or TCHAR


	//signed
	typedef char Byte;
	typedef signed char Int8;
	typedef short Int16;
	typedef int Int32;
	typedef __int64 Int64;

	//Unsigned
	typedef unsigned char UByte;
	typedef unsigned char UInt8;
	typedef unsigned short UInt16;
	typedef unsigned int UInt32;
	typedef unsigned __int64 UInt64;

	//types
	typedef UInt32 Count; ///< A counter variable
	typedef UInt32 Size; ///< Stores the size of an array etc
	typedef UInt16 Size16; ///< Stores the size of an array etc
	typedef UInt32 Index; //'0 base' index
	typedef UInt16 Index16; //'0 base' short index
	typedef UInt32 Handle; //'1 base' index where 0 is invalid
	typedef UInt8 Priority; //Priority value



	template <class _Type>
	inline const _Type& min( const _Type& a, const _Type& b )
	{ return a < b ? a : b; }
	
	template <class _Type>
	inline const _Type& max( const _Type& a, const _Type& b )
	{ return a > b ? a : b; }
	

} //END: LIDAR

#endif