
// map_editor.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'pch.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// MapEditorApp:
// See map_editor.cpp for the implementation of this class
//

class MapEditorApp : public CWinApp
{
public:
	MapEditorApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern MapEditorApp theApp;
