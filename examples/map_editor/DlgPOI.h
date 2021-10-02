#pragma once

#include "core/map.hpp"

// DlgPOI dialog

class DlgPOI : public CDialogEx
{
	DECLARE_DYNAMIC(DlgPOI)

public:
	DlgPOI(CWnd* pParent = nullptr);   // standard constructor
	virtual ~DlgPOI();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_POI };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
    afx_msg void OnBnClickedCancel();
    afx_msg void OnBnClickedOk();
    dg::Map* map = nullptr;
    unsigned __int64 ID;
    double lat;
    double lon;
    int floor;
    CString name;
    virtual BOOL OnInitDialog();
    CListBox m_listImages;
    afx_msg void OnSelchangeListImages();


    afx_msg void OnBnClickedBtnRegisterImage();
};
