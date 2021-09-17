// DlgNode.cpp : implementation file
//

#include "framework.h"
#include "MapEditorApp.h"
#include "DlgNode.h"
#include "afxdialogex.h"
#include <string>


// DlgNode dialog

IMPLEMENT_DYNAMIC(DlgNode, CDialogEx)

DlgNode::DlgNode(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_NODE, pParent)
    , ID(0)
    , type(0)
    , lat(0)
    , lon(0)
    , floor(0)
{

}

DlgNode::~DlgNode()
{
}

void DlgNode::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Text(pDX, IDC_EDIT_ID, ID);
    DDX_Text(pDX, IDC_EDIT_LAT, lat);
    DDX_Text(pDX, IDC_EDIT_LON, lon);
    DDX_Text(pDX, IDC_EDIT_FLOOR, floor);
    DDX_Control(pDX, IDC_LIST_TYPE, m_listType);
    DDX_Control(pDX, IDC_LIST_EDGES, m_listEdges);
}


BEGIN_MESSAGE_MAP(DlgNode, CDialogEx)
    ON_BN_CLICKED(IDCANCEL, &DlgNode::OnBnClickedCancel)
    ON_BN_CLICKED(IDOK, &DlgNode::OnBnClickedOk)
END_MESSAGE_MAP()


// DlgNode message handlers


void DlgNode::OnBnClickedOk()
{
    type = m_listType.GetCurSel();
    CDialogEx::OnOK();
}

void DlgNode::OnBnClickedCancel()
{
    CDialogEx::OnCancel();
}

BOOL DlgNode::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    m_listType.AddString("NODE_BASIC");
    m_listType.AddString("NODE_JUNCTION");
    m_listType.AddString("NODE_DOOR");
    m_listType.AddString("NODE_ELEVATOR");
    m_listType.AddString("NODE_ESCALATOR");
    m_listType.SetCurSel(type);

    for (auto it = edge_ids.begin(); it != edge_ids.end(); it++)
    {
        m_listEdges.AddString(std::to_string(*it).c_str());
    }    

    return TRUE;  // return TRUE unless you set the focus to a control
}

