#ifndef __DG_CURL__
#define __DG_CURL__

#define CURL_STATICLIB
#include "curl/curl.h" 
#ifdef _WIN32
#ifdef _DEBUG
#pragma comment(lib, "libcurl_a_debug.lib")   // curl-7.65.3.zip 
#else
#pragma comment(lib, "libcurl_a.lib")         // curl-7.65.3.zip 
#endif // _DEBUG
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Crypt32.lib")
#pragma comment(lib, "Wldap32.lib")
#pragma comment(lib, "Normaliz.lib")
#pragma execution_character_set( "utf-8" )
#include <atlstr.h> 
#endif	// _WIN32

#endif // End of '__DG_CURL__'
