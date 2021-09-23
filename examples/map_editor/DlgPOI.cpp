// DlgPOI.cpp : implementation file
//

#include "framework.h"
#include "MapEditorApp.h"
#include "DlgPOI.h"
#include "afxdialogex.h"


// DlgPOI dialog

IMPLEMENT_DYNAMIC(DlgPOI, CDialogEx)

DlgPOI::DlgPOI(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_POI, pParent)
	, ID(0)
	, lat(0)
	, lon(0)
	, floor(0)
	, name(_T(""))
{

}

DlgPOI::~DlgPOI()
{
}

void DlgPOI::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_ID, ID);
	DDX_Text(pDX, IDC_EDIT_LAT, lat);
	DDX_Text(pDX, IDC_EDIT_LON, lon);
	DDX_Text(pDX, IDC_EDIT_FLOOR, floor);
	DDX_Text(pDX, IDC_EDIT_NAME, name);
}


BEGIN_MESSAGE_MAP(DlgPOI, CDialogEx)
    ON_BN_CLICKED(IDCANCEL, &DlgPOI::OnBnClickedCancel)
	ON_BN_CLICKED(IDOK, &DlgPOI::OnBnClickedOk)
END_MESSAGE_MAP()


// DlgPOI message handlers


void DlgPOI::OnBnClickedCancel()
{
    // TODO: Add your control notification handler code here
    CDialogEx::OnCancel();
}


void DlgPOI::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	CDialogEx::OnOK();
}


BOOL DlgPOI::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	int delta = 30;
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

	return TRUE;  // return TRUE unless you set the focus to a control
}
