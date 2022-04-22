#pragma once


// DlgMapAlign dialog

class DlgMapAlign : public CDialogEx
{
	DECLARE_DYNAMIC(DlgMapAlign)

public:
	DlgMapAlign(CWnd* pParent = nullptr);   // standard constructor
	virtual ~DlgMapAlign();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MAP_ALIGN };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	void* m_mapEditor = nullptr;
	double m_map_ref_pixel_x;
	double m_map_ref_pixel_y;
	double m_map_pixel_per_meter;
	double m_map_image_rotation_deg;
	afx_msg void OnBnClickedBtnApply();
	afx_msg void OnBnClickedOk();
};
