 /*++

	VER_LEGALCOPYRIGHT_STR

Module Name:

    silabs_sal.h

Abstract:

    A top-level, across all repos, authoritative, Master include file for
	older IDEs that do not support the SAL-annotations that our API Header files use.

Environment:

    User mode

--*/
//
#if (_MSC_VER > 1000)
#pragma once
#endif

#ifndef	HOST_COMMON_INCLUDE_SILABS_SAL_H_INCLUDED_VASIQW4TVT
#define	HOST_COMMON_INCLUDE_SILABS_SAL_H_INCLUDED_VASIQW4TVT

#if ! defined(_Check_return_)
#define	_Check_return_
#endif // ! defined(_Check_return_)
#if ! defined(_Ret_range_)
#define _Ret_range_(lb,ub)
#endif // ! defined(_Ret_range_)
#if ! defined(_Success_)
#define	_Success_(expr)
#endif // ! defined(_Success_)
#if ! defined(_In_)
#define	_In_
#endif // ! defined(_In_)
#if ! defined(_In_opt_)
#define	_In_opt_
#endif // ! defined(_In_opt_)
#if ! defined(_Out_)
#define	_Out_
#endif // ! defined(_Out_)
#if ! defined(_In_range_)
#define _In_range_(lb,ub)
#endif // ! defined(_In_range_)
#if ! defined(_Out_range_)
#define _Out_range_(lb,ub)
#endif // ! defined(_Out_range_)
#if ! defined(_In_reads_bytes_)
#define	_In_reads_bytes_(n)
#endif // ! defined(_In_reads_bytes_)
#if ! defined(_Out_writes_bytes_)
#define	_Out_writes_bytes_(n)
#endif // ! defined(_Out_writes_bytes_)
#if ! defined(_Out_writes_bytes_opt_)
#define	_Out_writes_bytes_opt_(n)
#endif // ! defined(_Out_writes_bytes_opt_)
#if ! defined(_Inout_updates_bytes_opt_)
#define	_Inout_updates_bytes_opt_(n)
#endif // ! defined(_Inout_updates_bytes_opt_)
#if	! defined(_Printf_format_string_)
#define	_Printf_format_string_
#endif	// ! defined(_Printf_format_string_)
#if	! defined(_Use_decl_annotations_)
#define	_Use_decl_annotations_
#endif	// ! defined(_Use_decl_annotations_)
#if 	! defined(_Acquires_lock_)
#define _Acquires_lock_(arg)
#endif // ! defined(_Acquires_lock_)
#if	! defined(_Releases_lock_)
#define _Releases_lock_(arg)
#endif // !defined(_Releases_lock_)
#if	! defined(_Pre_defensive_)
#define _Pre_defensive_
#endif

#endif // !defined(HOST_COMMON_INCLUDE_SILABS_SAL_H_INCLUDED_VASIQW4TVT)

