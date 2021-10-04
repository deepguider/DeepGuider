// DlgEdge.cpp : implementation file
//

#include "framework.h"
#include "MapEditorApp.h"
#include "DlgEdge.h"
#include "core/map.hpp"
#include "afxdialogex.h"


// DlgEdge dialog

IMPLEMENT_DYNAMIC(DlgEdge, CDialogEx)

DlgEdge::DlgEdge(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_EDGE, pParent)
	, ID(0)
	, node_id1(0)
	, node_id2(0)
	, type(0)
	, length(0)
	, directed(FALSE)
	, lr_side(-1)
{

}

DlgEdge::~DlgEdge()
{
}

void DlgEdge::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Text(pDX, IDC_EDIT_ID, ID);
    DDX_Control(pDX, IDC_LIST_TYPE, m_listType);
    DDX_Text(pDX, IDC_EDIT_NODE1, node_id1);
    DDX_Text(pDX, IDC_EDIT_NODE2, node_id2);
    DDX_Text(pDX, IDC_EDIT_LENGTH, length);
    DDX_Text(pDX, IDC_EDIT_DIRECTED, directed);
    DDX_Control(pDX, IDC_LIST_LR, m_listLR);
}


BEGIN_MESSAGE_MAP(DlgEdge, CDialogEx)
    ON_BN_CLICKED(IDOK, &DlgEdge::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &DlgEdge::OnBnClickedCancel)
END_MESSAGE_MAP()


// DlgEdge message handlers


void DlgEdge::OnBnClickedOk()
{
	type = m_listType.GetCurSel();
	int lr = m_listLR.GetCurSel();
	if (lr == 0) lr_side = dg::Edge::LR_NONE;
	else if (lr == 1) lr_side = dg::Edge::LR_LEFT;
	else if (lr == 2) lr_side = dg::Edge::LR_RIGHT;

    CDialogEx::OnOK();
}


void DlgEdge::OnBnClickedCancel()
{
	CDialogEx::OnCancel();
}


BOOL DlgEdge::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	int delta = 80;
	POINT pt;
	GetCursorPos(&pt);
	RECT rc;
	GetWindowRect(&rc);
	int w = rc.right - rc.left;
	int h = rc.bottom - rc.top;
	int x = pt.x - w - delta;
	int y = pt.y - h * 2 / 3;
	if (x <= 0) x = pt.x + delta;
	if (y <= 0) y = delta;
	SetWindowPos(NULL, x, y, 0, 0, SWP_NOSIZE | SWP_NOZORDER);

	m_listLR.AddString("LR_NONE");
	m_listLR.AddString("LR_LEFT");
	m_listLR.AddString("LR_RIGHT");
	if (lr_side == dg::Edge::LR_NONE) m_listLR.SetCurSel(0);
	else if (lr_side == dg::Edge::LR_LEFT) m_listLR.SetCurSel(1);
	else if (lr_side == dg::Edge::LR_RIGHT) m_listLR.SetCurSel(2);

	m_listType.AddString("EDGE_SIDEWALK");
	m_listType.AddString("EDGE_ROAD");
	m_listType.AddString("EDGE_CROSSWALK");
	m_listType.AddString("EDGE_ELEVATOR");
	m_listType.AddString("EDGE_ESCALATOR");
	m_listType.AddString("EDGE_STAIR");
	m_listType.AddString("EDGE_DOORWAY");
	m_listType.SetCurSel(type);

	return TRUE;  // return TRUE unless you set the focus to a control
}

BOOL DlgEdge::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN)
	{
		if (pMsg->wParam == VK_DELETE)
		{
			erase = true;
			CDialogEx::OnOK();
		}
	}
	return CDialogEx::PreTranslateMessage(pMsg);
}
