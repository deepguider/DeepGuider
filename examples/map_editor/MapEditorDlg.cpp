
// MapEditorDlg.cpp : implementation file
//

#include "framework.h"
#include "MapEditorApp.h"
#include "MapEditorDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// MapEditorDlg dialog



MapEditorDlg::MapEditorDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_MAP_EDITOR_DIALOG, pParent)
    , m_show_poi(FALSE)
    , m_show_streetview(FALSE)
	, m_show_error(FALSE)
	, m_show_lrside(FALSE)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void MapEditorDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Check(pDX, IDC_CHECK_POI, m_show_poi);
    DDX_Check(pDX, IDC_CHECK_STREETVIEW, m_show_streetview);
    DDX_Check(pDX, IDC_CHECK_SHOW_ERROR, m_show_error);
    DDX_Check(pDX, IDC_CHECK_LRPose, m_show_lrside);
}

BEGIN_MESSAGE_MAP(MapEditorDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDOK, &MapEditorDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &MapEditorDlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON_ETRI, &MapEditorDlg::OnBnClickedButtonEtri)
	ON_BN_CLICKED(IDC_BUTTON_COEX, &MapEditorDlg::OnBnClickedButtonCoex)
	ON_BN_CLICKED(IDC_BUTTON_BUCHEON, &MapEditorDlg::OnBnClickedButtonBucheon)
	ON_BN_CLICKED(IDC_BUTTON_SAVE, &MapEditorDlg::OnBnClickedButtonSave)
	ON_BN_CLICKED(IDC_BUTTON_SAVE_AS, &MapEditorDlg::OnBnClickedButtonSaveAs)
	ON_BN_CLICKED(IDC_BUTTON_LOAD, &MapEditorDlg::OnBnClickedButtonLoad)
    ON_BN_CLICKED(IDC_CHECK_POI, &MapEditorDlg::OnBnClickedCheckPoi)
	ON_BN_CLICKED(IDC_CHECK_STREETVIEW, &MapEditorDlg::OnBnClickedCheckStreetview)
	ON_BN_CLICKED(IDC_BUTTON_DOWNLOAD, &MapEditorDlg::OnBnClickedButtonDownload)
    ON_BN_CLICKED(IDC_BUTTON_VERIFY_MAP, &MapEditorDlg::OnBnClickedButtonVerifyMap)
	ON_BN_CLICKED(IDC_CHECK_SHOW_ERROR, &MapEditorDlg::OnBnClickedCheckShowError)
    ON_BN_CLICKED(IDC_BUTTON_EXPORT_TO_JSON, &MapEditorDlg::OnBnClickedButtonExportToJson)
	ON_BN_CLICKED(IDC_BUTTON_FIX_MAP_ERROR, &MapEditorDlg::OnBnClickedButtonFixMapError)
	ON_BN_CLICKED(IDC_CHECK_LRPose, &MapEditorDlg::OnBnClickedCheckLrpose)
	ON_BN_CLICKED(IDC_BUTTON_UPDATE_LR, &MapEditorDlg::OnBnClickedButtonUpdateLr)
	ON_BN_CLICKED(IDC_BTN_ADJUST_MAP_ALIGN, &MapEditorDlg::OnBnClickedBtnAdjustMapAlign)
END_MESSAGE_MAP()


// MapEditorDlg message handlers

BOOL MapEditorDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	SetWindowPos(NULL, 70, 10, 0, 0, SWP_NOSIZE | SWP_NOZORDER);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void MapEditorDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

void MapEditorDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

HCURSOR MapEditorDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void MapEditorDlg::OnBnClickedOk()
{
	if (m_editor)
	{
		m_editor->stop();
		cv::destroyAllWindows();
		delete m_editor;
	}
	CDialogEx::OnOK();
}


void MapEditorDlg::OnBnClickedCancel()
{
	if (m_editor)
	{
		m_editor->stop();
		cv::destroyAllWindows();
		delete m_editor;
	}
	CDialogEx::OnCancel();
}


void MapEditorDlg::OnBnClickedButtonEtri()
{
	runEditor("etri");
}

void MapEditorDlg::OnBnClickedButtonCoex()
{
	runEditor("coex");
}

void MapEditorDlg::OnBnClickedButtonBucheon()
{
	runEditor("bucheon");
}

void MapEditorDlg::runEditor(std::string site)
{
	if (m_editor)
	{
		m_editor->stop();
		delete m_editor;
	}
	m_show_poi = FALSE;
	m_show_streetview = FALSE;
	m_show_lrside = FALSE;
	UpdateData(FALSE);
	m_editor = new MapEditor();
	m_editor->configure(site);
	m_editor->run();
}


void MapEditorDlg::OnBnClickedButtonSave()
{
	if (m_editor) m_editor->save();
}


void MapEditorDlg::OnBnClickedButtonSaveAs()
{
	if (m_editor) m_editor->saveAs();
}


void MapEditorDlg::OnBnClickedButtonLoad()
{
	if (m_editor) m_editor->load();
}


void MapEditorDlg::OnBnClickedCheckPoi()
{
	UpdateData();
	if (m_editor) m_editor->showPoi(m_show_poi == TRUE);
}


void MapEditorDlg::OnBnClickedCheckStreetview()
{
	UpdateData();
	if (m_editor) m_editor->showStreetView(m_show_streetview == TRUE);
}


void MapEditorDlg::OnBnClickedCheckLrpose()
{
	UpdateData();
	if (m_editor) m_editor->showLRSide(m_show_lrside == TRUE);
}


void MapEditorDlg::OnBnClickedButtonDownload()
{
	BeginWaitCursor();
	if (m_editor) m_editor->download();
	EndWaitCursor();
}


void MapEditorDlg::OnBnClickedButtonVerifyMap()
{
	BeginWaitCursor();
	if (m_editor)
	{
		m_editor->verify();
		m_editor->showMapError(true);
		m_show_error = TRUE;
		UpdateData(FALSE);
	}
	EndWaitCursor();
}


void MapEditorDlg::OnBnClickedButtonFixMapError()
{
	BeginWaitCursor();
	if (m_editor)
	{
		m_editor->fixMapError();
	}
	EndWaitCursor();
}


void MapEditorDlg::OnBnClickedCheckShowError()
{
	UpdateData();
	if (m_editor) m_editor->showMapError(m_show_error == TRUE);
}


void MapEditorDlg::OnBnClickedButtonUpdateLr()
{
	BeginWaitCursor();
	if (m_editor)
	{
		m_editor->updateLRSide();
		m_editor->showLRSide(true);
		m_show_lrside = TRUE;
		UpdateData(FALSE);
	}
	EndWaitCursor();
}


void MapEditorDlg::OnBnClickedButtonExportToJson()
{
	BeginWaitCursor();
	if (m_editor) m_editor->exportToJson();
	EndWaitCursor();
}


void MapEditorDlg::OnBnClickedBtnAdjustMapAlign()
{
	if (m_editor) m_editor->adjustMapAlign();
}
