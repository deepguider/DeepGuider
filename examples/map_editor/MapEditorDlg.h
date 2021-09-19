
// MapEditorDlg.h : header file
//

#pragma once

#include "MapEditor.h"

// MapEditorDlg dialog
class MapEditorDlg : public CDialogEx
{
// Construction
public:
	MapEditorDlg(CWnd* pParent = nullptr);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MAP_EDITOR_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;
	MapEditor* m_editor = nullptr;

	void runEditor(std::string site);

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
    afx_msg void OnBnClickedOk();
    afx_msg void OnBnClickedCancel();
    afx_msg void OnBnClickedButtonEtri();
    afx_msg void OnBnClickedButtonCoex();
    afx_msg void OnBnClickedButtonBucheon();
	afx_msg void OnBnClickedButtonSave();
	afx_msg void OnBnClickedButtonSaveAs();
	afx_msg void OnBnClickedButtonLoad();
    BOOL m_show_poi;
    BOOL m_show_streetview;
    afx_msg void OnBnClickedCheckPoi();
    afx_msg void OnBnClickedCheckStreetview();
    afx_msg void OnBnClickedButtonDownload();
};
