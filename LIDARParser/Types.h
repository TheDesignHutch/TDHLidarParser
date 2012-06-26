#ifndef LIDARPARSER_TYPES_H
#define LIDARPARSER_TYPES_H

#define NOMINMAX
#include <TCHAR.h>

#define USE_TCHAR //< Set this to enable project specific unicode/utf8 build
#ifdef USE_TCHAR
#	define RTCHAR TChar
#else
#	define RTCHAR Char8
#endif

namespace LIDAR
{		
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