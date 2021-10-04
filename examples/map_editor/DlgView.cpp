// DlgView.cpp : implementation file
//

#include "framework.h"
#include "MapEditorApp.h"
#include "DlgView.h"
#include "afxdialogex.h"


// DlgView dialog

IMPLEMENT_DYNAMIC(DlgView, CDialogEx)

DlgView::DlgView(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_VIEW, pParent)
	, ID(0)
	, lat(0)
	, lon(0)
	, floor(0)
	, heading(0)
	, date(_T(""))
{

}

DlgView::~DlgView()
{
}

void DlgView::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_ID, ID);
	DDX_Text(pDX, IDC_EDIT_LAT, lat);
	DDX_Text(pDX, IDC_EDIT_LON, lon);
	DDX_Text(pDX, IDC_EDIT_FLOOR, floor);
	DDX_Text(pDX, IDC_EDIT_HEADING, heading);
	DDX_Text(pDX, IDC_EDIT_DATE, date);
}


BEGIN_MESSAGE_MAP(DlgView, CDialogEx)
    ON_BN_CLICKED(IDOK, &DlgView::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &DlgView::OnBnClickedCancel)
END_MESSAGE_MAP()


// DlgView message handlers


void DlgView::OnBnClickedOk()
{
    CDialogEx::OnOK();
}


void DlgView::OnBnClickedCancel()
{
	CDialogEx::OnCancel();
}


BOOL DlgView::OnInitDialog()
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

BOOL DlgView::PreTranslateMessage(MSG* pMsg)
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
