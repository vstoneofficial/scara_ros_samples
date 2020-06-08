 /*++

	VER_LEGALCOPYRIGHT_STR

Module Name:

    silabs_defs.h

Abstract:

    A top-level, across-all-repos, collection of some "useful" defines/enums

Environment:

    User or kernel mode

--*/
//
#if (_MSC_VER > 1000)
#pragma once
#endif

#ifndef	HOST_COMMON_INCLUDE_SILABS_DEFS_H_INCLUDED_BVHHTNCO7E
#define	HOST_COMMON_INCLUDE_SILABS_DEFS_H_INCLUDED_BVHHTNCO7E

#include	"silabs_sal.h"

#if	! defined(__cplusplus)
#define bool BOOLEAN
#endif // ! defined(__cplusplus)

#if defined(DEFINE_GUID)
DEFINE_GUID(GUID_DEVINTERFACE_SILABS_CP210x,
	0xa2a39220, 0x39f4, 0x4b88, 0xae, 0xcb, 0x3d, 0x86, 0xa3, 0x5d, 0xc7, 0x48);

// USBXpress
// {3C5E1462-5695-4e18-876B-F3F3D08AAF18}
DEFINE_GUID(GUID_DEVINTERFACE_SILABS_USBXPRESS_BRIDGE, 
0x3c5e1462, 0x5695, 0x4e18, 0x87, 0x6b, 0xf3, 0xf3, 0xd0, 0x8a, 0xaf, 0x18);
#endif // DEFINE_GUID

#define SILABS_TEST_FILL	0xa5	// 10100101


// Company Vendor ID (VID)
typedef enum _SILABS_VID {
	SILABS_VID_10C4 = ((unsigned short)(0xFFFF & 0x10C4))	// Decimal 4292; VID aquired via Cygnal.
	, SILABS_VID_1BA4 = ((unsigned short)(0xFFFF & 0x1BA4))	// Decimal 7076; VID aquired via Ember.
	, SILABS_VID_2544 = ((unsigned short)(0xFFFF & 0x2544))	// Decimal 9540; VID aquired via Energy Micro.
} SILABS_VID, *PSILABS_VID;
_Check_return_
_Success_(return == TRUE)
__inline static bool IsValidSILABS_VID ( _In_ const SILABS_VID _v ) { return (SILABS_VID_10C4 == _v); }

// Device Product IDs (PIDs)
typedef enum _SILABS_PID {
	SILABS_PID_UNKNOWN		=  ((unsigned short)(0xFFFF & 0x0000))
	, SILABS_PID_CP210SINGLEPORT		= ((unsigned short)(0xFFFF & 0xEA60))
	, SILABS_PID_CP210SINGLEPORTII		= ((unsigned short)(0xFFFF & 0xEA63))
	, SILABS_PID_CP2101		= SILABS_PID_CP210SINGLEPORT
	, SILABS_PID_CP2102		= SILABS_PID_CP210SINGLEPORT
	, SILABS_PID_CP2102N	= SILABS_PID_CP210SINGLEPORT
	, SILABS_PID_CP2103		= SILABS_PID_CP210SINGLEPORT
	, SILABS_PID_CP2104		= SILABS_PID_CP210SINGLEPORT
	, SILABS_PID_CP210DUALPORT		= ((unsigned short)(0xFFFF & 0xEA70))
	, SILABS_PID_CP210DUALPORTII	= ((unsigned short)(0xFFFF & 0xEA7A))
	, SILABS_PID_CP2105		= SILABS_PID_CP210DUALPORT
	, SILABS_PID_CP2105II	= SILABS_PID_CP210DUALPORTII
	, SILABS_PID_CP210QUADPORT		= ((unsigned short)(0xFFFF & 0xEA71))
	, SILABS_PID_CP210QUADPORTII	= ((unsigned short)(0xFFFF & 0xEA7B))
	, SILABS_PID_CP2108		= SILABS_PID_CP210QUADPORT
	, SILABS_PID_CP2108II	= SILABS_PID_CP210QUADPORTII
	, SILABS_PID_CP2109		= SILABS_PID_CP210SINGLEPORT
	, SILABS_PID_CP2110		= ((unsigned short)(0xFFFF & 0xEA80))
	, SILABS_PID_CP2111		= SILABS_PID_CP2110
	, SILABS_PID_CP2112		= ((unsigned short)(0xFFFF & 0xEA90))
	, SILABS_PID_CP2114		= ((unsigned short)(0xFFFF & 0xEAB0))
	, SILABS_PID_CP2130		= ((unsigned short)(0xFFFF & 0x87A0))
	, SILABS_PID_USBXPress	= ((unsigned short)(0xFFFF & 0xEA61))
} SILABS_PID, *PSILABS_PID;
_Check_return_
_Success_(return == TRUE)
__inline static bool IsValidSILABS_PID(_In_ const SILABS_PID _p) {
#pragma warning ( push )
#pragma warning ( disable : 6287 ) // warning C6287: redundant code: the left and right sub-expressions are identical
	return ((SILABS_PID_CP210SINGLEPORT == _p) || (SILABS_PID_CP210SINGLEPORTII == _p)
		 || (SILABS_PID_CP210DUALPORT == _p) || (SILABS_PID_CP210DUALPORTII == _p)
		 || (SILABS_PID_CP210QUADPORT == _p) || (SILABS_PID_CP210QUADPORTII == _p)
		 || (SILABS_PID_CP2101 == _p) || (SILABS_PID_CP2102 == _p) || (SILABS_PID_CP2102N == _p) || (SILABS_PID_CP2103 == _p)
		 || (SILABS_PID_CP2104 == _p)
		 || (SILABS_PID_CP2105 == _p) || (SILABS_PID_CP2105II == _p)
		 || (SILABS_PID_CP2108 == _p) || (SILABS_PID_CP2108II == _p)
		 || (SILABS_PID_CP2109 == _p)
		 || (SILABS_PID_CP2110 == _p) || (SILABS_PID_CP2111 == _p) || (SILABS_PID_CP2112 == _p) || (SILABS_PID_CP2114 == _p)
		 || (SILABS_PID_CP2130 == _p) || (SILABS_PID_USBXPress == _p));
#pragma warning ( pop )
}
_Check_return_
_Success_(return == TRUE)
__inline static bool IsValidCP210X_PID(_In_ const SILABS_PID _p) {
#pragma warning ( push )
#pragma warning ( disable : 6287 ) // warning C6287: redundant code: the left and right sub-expressions are identical
	return ((SILABS_PID_CP210SINGLEPORT == _p) || (SILABS_PID_CP210SINGLEPORTII == _p)
		 || (SILABS_PID_CP210DUALPORT == _p) || (SILABS_PID_CP210DUALPORTII == _p)
		 || (SILABS_PID_CP210QUADPORT == _p) || (SILABS_PID_CP210QUADPORTII == _p)
		 || (SILABS_PID_CP2101 == _p) || (SILABS_PID_CP2102 == _p) || (SILABS_PID_CP2102N == _p) || (SILABS_PID_CP2103 == _p)
		 || (SILABS_PID_CP2104 == _p)
		 || (SILABS_PID_CP2105 == _p) || (SILABS_PID_CP2105II == _p)
		 || (SILABS_PID_CP2108 == _p) || (SILABS_PID_CP2108II == _p)
		 || (SILABS_PID_CP2109 == _p));
#pragma warning ( pop )
}

// Device Part Numbers
typedef enum _CP210X_PARTNUM {
	CP210x_PARTNUM_UNKNOWN =  ((BYTE)(0xFF & 0x00))
	, CP210x_PARTNUM_CP2101 = ((BYTE)(0xFF & 0x01))
	, CP210x_PARTNUM_CP2102 = ((BYTE)(0xFF & 0x02))
	, CP210x_PARTNUM_CP2103 = ((BYTE)(0xFF & 0x03))
	, CP210x_PARTNUM_CP2104 = ((BYTE)(0xFF & 0x04))
	, CP210x_PARTNUM_CP2105 = ((BYTE)(0xFF & 0x05))
	, CP210x_PARTNUM_CP2108 = ((BYTE)(0xFF & 0x08))
	, CP210x_PARTNUM_CP2109 = ((BYTE)(0xFF & 0x09))
	, CP210x_PARTNUM_CP2102N_QFN28 = ((BYTE)(0xFF & 0x20))
	, CP210x_PARTNUM_CP2102N_QFN24 = ((BYTE)(0xFF & 0x21))
	, CP210x_PARTNUM_CP2102N_QFN20 = ((BYTE)(0xFF & 0x22))
	, CP210x_PARTNUM_USBXPRESS_F3XX = ((BYTE)(0xFF & 0x80))
	, CP210x_PARTNUM_USBXPRESS_EFM8 = ((BYTE)(0xFF & 0x80))
	, CP210x_PARTNUM_USBXPRESS_EFM32 = ((BYTE)(0xFF & 0x81))
} CP210X_PARTNUM, *PCP210X_PARTNUM;
_Check_return_
_Success_(return == TRUE)
__inline static bool IsValidCP210X_PARTNUM(_In_ const CP210X_PARTNUM _v) {
	return (((CP210x_PARTNUM_CP2101 <= _v) && (_v <= CP210x_PARTNUM_CP2105)) || (CP210x_PARTNUM_CP2108 == _v) || (CP210x_PARTNUM_CP2109 == _v) || ((CP210x_PARTNUM_CP2102N_QFN28 <= _v) && (_v <= CP210x_PARTNUM_CP2102N_QFN20)) || (CP210x_PARTNUM_USBXPRESS_F3XX == _v));
}
_Check_return_
_Success_(return == TRUE)
__inline static bool IsOTPCP210X_PARTNUM(_In_ const CP210X_PARTNUM _v) {
	return ((CP210x_PARTNUM_CP2104 == _v) || (CP210x_PARTNUM_CP2105 == _v) || (CP210x_PARTNUM_CP2109 == _v));
}
_Check_return_
_Success_(return == TRUE)
__inline static bool IsOTP(_In_ const SILABS_PID _p, _In_ const CP210X_PARTNUM _v) {
	return IsValidCP210X_PID(_p) ? IsOTPCP210X_PARTNUM(_v) : ((SILABS_PID_CP2110 == _v) || (SILABS_PID_CP2112 == _v) || (SILABS_PID_CP2114 == _v) || (SILABS_PID_CP2130 == _v));
}
_Check_return_
_Success_(return == TRUE)
__inline static bool IsWriteReadLatchPartNum(_In_ const CP210X_PARTNUM _v) {
	return ((CP210x_PARTNUM_CP2102N_QFN28 == _v) || (CP210x_PARTNUM_CP2102N_QFN24 == _v) || (CP210x_PARTNUM_CP2102N_QFN20 == _v) || (CP210x_PARTNUM_CP2103 == _v) || (CP210x_PARTNUM_CP2104 == _v) || (CP210x_PARTNUM_CP2105 == _v) || (CP210x_PARTNUM_CP2108 == _v));
}

// Return Codes ??Of what?? API DLLs?
typedef enum _SILABS_STATUS {
	SILABS_STATUS_SUCCESS			= ((BYTE)(0xFF & 0x00))
	, SILABS_STATUS_TBD				= ((BYTE)(0xFF & 0x01))
	, SILABS_STATUS_UNKNOWN_ERROR	= ((BYTE)(0xFF & 0xFF))
} SILABS_STATUS, *PSILABS_STATUS;
_Check_return_
_Success_(return == TRUE)
__inline static bool IsValidSILABS_STATUS(_In_ const SILABS_STATUS _s) {
	return (((SILABS_STATUS_SUCCESS <= _s) && (_s <= SILABS_STATUS_TBD)) || (SILABS_STATUS_UNKNOWN_ERROR == _s));
}
_Check_return_
_Success_(return == TRUE)
__inline static bool IsSuccessSILABS_STATUS(_In_ const SILABS_STATUS _s) { return SILABS_STATUS_SUCCESS == _s; }

#endif // !defined(HOST_COMMON_INCLUDE_SILABS_DEFS_H_INCLUDED_BVHHTNCO7E)

