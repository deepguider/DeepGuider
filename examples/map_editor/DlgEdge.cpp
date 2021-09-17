// DlgEdge.cpp : implementation file
//

#include "framework.h"
#include "MapEditorApp.h"
#include "DlgEdge.h"
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
	, lr_side(0)
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
	DDX_Text(pDX, IDC_EDIT_LR, lr_side);
}


BEGIN_MESSAGE_MAP(DlgEdge, CDialogEx)
    ON_BN_CLICKED(IDOK, &DlgEdge::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &DlgEdge::OnBnClickedCancel)
END_MESSAGE_MAP()


// DlgEdge message handlers


void DlgEdge::OnBnClickedOk()
{
	type = m_listType.GetCurSel();
    CDialogEx::OnOK();
}


void DlgEdge::OnBnClickedCancel()
{
	CDialogEx::OnCancel();
}


BOOL DlgEdge::OnInitDialog()
{
	CDialogEx::OnInitDialog();

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
