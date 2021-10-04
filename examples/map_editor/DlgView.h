#pragma once

// DlgView dialog

class DlgView : public CDialogEx
{
	DECLARE_DYNAMIC(DlgView)

public:
	DlgView(CWnd* pParent = nullptr);   // standard constructor
	virtual ~DlgView();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_VIEW };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
	unsigned __int64 ID;
	double lat;
	double lon;
	int floor;
	double heading;
	CString date;
	virtual BOOL OnInitDialog();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	bool erase = false;
};
