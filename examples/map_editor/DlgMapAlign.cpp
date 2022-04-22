// DlgMapAlign.cpp : implementation file
//

#include "framework.h"
#include "MapEditorApp.h"
#include "DlgMapAlign.h"
#include "MapEditor.h"
#include "afxdialogex.h"


// DlgMapAlign dialog

IMPLEMENT_DYNAMIC(DlgMapAlign, CDialogEx)

DlgMapAlign::DlgMapAlign(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_MAP_ALIGN, pParent)
	, m_map_ref_pixel_x(0)
	, m_map_ref_pixel_y(0)
	, m_map_pixel_per_meter(0)
	, m_map_image_rotation_deg(0)
{

}

DlgMapAlign::~DlgMapAlign()
{
}

void DlgMapAlign::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_IMAGE_REF_X, m_map_ref_pixel_x);
	DDX_Text(pDX, IDC_EDIT_IMAGE_REF_Y, m_map_ref_pixel_y);
	DDX_Text(pDX, IDC_EDIT_PIXEL_PER_METER, m_map_pixel_per_meter);
	DDX_Text(pDX, IDC_EDIT_IMAGE_ROTATION, m_map_image_rotation_deg);
}


BEGIN_MESSAGE_MAP(DlgMapAlign, CDialogEx)
	ON_BN_CLICKED(IDC_BTN_APPLY, &DlgMapAlign::OnBnClickedBtnApply)
	ON_BN_CLICKED(IDOK, &DlgMapAlign::OnBnClickedOk)
END_MESSAGE_MAP()


// DlgMapAlign message handlers
void DlgMapAlign::OnBnClickedBtnApply()
{
	UpdateData(TRUE);
	MapEditor* editor = (MapEditor *)m_mapEditor;
	editor->applyMapAlign(m_map_ref_pixel_x, m_map_ref_pixel_y, m_map_pixel_per_meter, m_map_image_rotation_deg);
}


void DlgMapAlign::OnBnClickedOk()
{
	delete this;
}
