#pragma once

#include <vector>

// DlgNode dialog

class DlgNode : public CDialogEx
{
	DECLARE_DYNAMIC(DlgNode)

public:
	DlgNode(CWnd* pParent = nullptr);   // standard constructor
	virtual ~DlgNode();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_NODE };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedCancel();
	unsigned __int64 ID;
	int type;
	double lat;
	double lon;
	int floor;
	std::vector<unsigned __int64> edge_ids;
	CListBox m_listType;
	CListBox m_listEdges;
	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedOk();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	bool erase = false;
};
