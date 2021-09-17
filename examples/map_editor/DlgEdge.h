#pragma once


// DlgEdge dialog

class DlgEdge : public CDialogEx
{
	DECLARE_DYNAMIC(DlgEdge)

public:
	DlgEdge(CWnd* pParent = nullptr);   // standard constructor
	virtual ~DlgEdge();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_EDGE };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
	unsigned __int64 ID;
	CListBox m_listType;
	unsigned __int64 node_id1;
	unsigned __int64 node_id2;
	int type;
	int lr_side;
	double length;
	BOOL directed;
	virtual BOOL OnInitDialog();
};
