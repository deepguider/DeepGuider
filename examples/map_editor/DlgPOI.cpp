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
	DDX_Control(pDX, IDC_LIST_IMAGES, m_listImages);
}


BEGIN_MESSAGE_MAP(DlgPOI, CDialogEx)
	ON_BN_CLICKED(IDCANCEL, &DlgPOI::OnBnClickedCancel)
	ON_BN_CLICKED(IDOK, &DlgPOI::OnBnClickedOk)
	ON_LBN_SELCHANGE(IDC_LIST_IMAGES, &DlgPOI::OnSelchangeListImages)
	ON_BN_CLICKED(IDC_BTN_REGISTER_IMAGE, &DlgPOI::OnBnClickedBtnRegisterImage)
END_MESSAGE_MAP()


// DlgPOI message handlers

void DlgPOI::OnBnClickedCancel()
{
	cv::destroyWindow("POI Image");
	CDialogEx::OnCancel();
}


void DlgPOI::OnBnClickedOk()
{
	cv::destroyWindow("POI Image");
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

	if (map)
	{
		std::vector<std::string> image_names = map->getNameOfRegisteredPOIImages(ID);
		for (auto it = image_names.begin(); it != image_names.end(); it++)
		{
			m_listImages.AddString(it->c_str());
		}
	}

	return TRUE;  // return TRUE unless you set the focus to a control
}


void DlgPOI::OnSelchangeListImages()
{
	if (map == nullptr) return;
	int index = m_listImages.GetCurSel();
	if (index == LB_ERR) return;
	cv::Mat image = map->getRegisteredPOIImage(ID, index);
	if (!image.empty())
	{
		cv::namedWindow("POI Image", cv::WINDOW_NORMAL);
		cv::setWindowProperty("POI Image", cv::WND_PROP_TOPMOST, 1);
		cv::imshow("POI Image", image);
		cv::waitKey(1);
	}
}


void DlgPOI::OnBnClickedBtnRegisterImage()
{
	char szFile[1024] = "";  // buffer for file path
	CString szFilter = "Image Files|*.bmp;*.png;*.jpg|All Files (*.*)|*.*||";
	CFileDialog dlg(TRUE, 0, szFile, OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST, szFilter, NULL);
	if (dlg.DoModal() == IDOK)
	{
		std::string fpath = CT2A(dlg.GetPathName().GetString());
		cv::Mat image = cv::imread(fpath);
		std::string image_name;
		if (!image.empty()) map->registerPOIImage(ID, image, image_name);
		m_listImages.AddString(image_name.c_str());
	}
}
