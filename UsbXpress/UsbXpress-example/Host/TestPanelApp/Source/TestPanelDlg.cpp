// TestPanelDlg.cpp : implementation file
//

#include "stdafx.h"
#include "TestPanel.h"
#include "TestPanelDlg.h"
#include "SelectDialog.h"
#include "siusbxp.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTestPanelDlg dialog

CTestPanelDlg::CTestPanelDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CTestPanelDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CTestPanelDlg)
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CTestPanelDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CTestPanelDlg)
	DDX_Control(pDX, IDC_AN1_METER, m_Analog_Meter1);
	DDX_Control(pDX, IDC_AN2_METER, m_Analog_Meter2);
	DDX_Control(pDX, IDC_BTN1_LED, m_ButtonLED1);
	DDX_Control(pDX, IDC_BTN2_LED, m_ButtonLED2);
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CTestPanelDlg, CDialog)
	//{{AFX_MSG_MAP(CTestPanelDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTestPanelDlg message handlers

BOOL CTestPanelDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	m_bReadError = FALSE;
	m_bWriteError = FALSE;

	// Add "About..." menu item to system menu.
	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
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
	
	// Custom Initialization follows:
	// Initialize handles
	m_hUSBDevice = INVALID_HANDLE_VALUE;

	// LED's
	CWnd *pWndLedBtn1 = (CWnd *)GetDlgItem(IDC_BTN1_LED);
	m_ButtonLED1.SetLED(pWndLedBtn1,ID_LED_GREEN,ID_SHAPE_ROUND,500);
	m_ButtonLED1.SwitchOn();
	m_ButtonLED1.LedOff();

	CWnd *pWndLedBtn2 = (CWnd *)GetDlgItem(IDC_BTN2_LED);
	m_ButtonLED2.SetLED(pWndLedBtn2,ID_LED_GREEN,ID_SHAPE_ROUND,500);
	m_ButtonLED2.SwitchOn();
	m_ButtonLED2.LedOff();

	// Analog Meters
	CString sUnitPot = "Pot";
	m_Analog_Meter1.SetUnits(sUnitPot);
	m_Analog_Meter1.SetRange(0,255);
	m_Analog_Meter1.UpdateNeedle(0);
	CString sUnitTemp = "Temp";
	m_Analog_Meter2.SetUnits(sUnitTemp);
	m_Analog_Meter2.SetRange(0,255);
	m_Analog_Meter2.UpdateNeedle(0);

	if (!GetDeviceList())
	{	
		AfxMessageBox("Error finding USB device. Aborting application",MB_OK|MB_ICONEXCLAMATION);
		OnCancel();
		return TRUE;
	}

	m_nDeviceNum = -1;

	if (!SelectDevice())
		return TRUE;

	// Set read write timeouts
	SI_SetTimeouts(5000, 5000);

	// Open device handle
	SI_STATUS status = SI_Open((DWORD)m_nDeviceNum, &m_hUSBDevice);

	if (status != SI_SUCCESS)
	{
		CString sMessage;
		sMessage.Format("Error opening device: %s\n\nApplication is aborting.\nReset hardware and try again.", m_sDeviceName);
		if (AfxMessageBox(sMessage,MB_OK|MB_ICONEXCLAMATION))
		{
			OnCancel();
			return TRUE;
		}
	}

	memset(&m_IObuffer, 0, sizeof(USB_iobuf));

	SetTimer(1,100,NULL);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CTestPanelDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.
void CTestPanelDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

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
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CTestPanelDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

void CTestPanelDlg::OnOKExit() 
{
	KillTimer(1);	

	SI_Close(m_hUSBDevice);

	CDialog::OnOK();
}

// Note: No cancel button exists on the dialog, but this function is used
// when dialog is terminated at customer request from message box input.
void CTestPanelDlg::OnCancel() 
{
	KillTimer(1);		

	SI_SetTimeouts(INFINITE, INFINITE);
	SI_Close(m_hUSBDevice); 

	//CDialog::OnCancel();
}


void CTestPanelDlg::OnTimer(UINT nIDEvent) 
{
	SI_STATUS status = SI_SUCCESS;

	// Confirm an error hasn't already occurred
	if ( (m_bReadError == TRUE) || (m_bWriteError == TRUE) )
	{
		// Call base timer
		CDialog::OnTimer(nIDEvent);

		// Further processing not possible
		return;		
	}

	// Obtain current values of dialog editable items
	m_IObuffer.led1 = ((CButton*) GetDlgItem(IDC_CHECK_LED1))->GetCheck(); 
	m_IObuffer.led2 = ((CButton*) GetDlgItem(IDC_CHECK_LED2))->GetCheck(); 

	unsigned char B0 = ((CButton*) GetDlgItem(IDC_P1_B0))->GetCheck();
	unsigned char B1 = ((CButton*) GetDlgItem(IDC_P1_B1))->GetCheck();
	unsigned char B2 = ((CButton*) GetDlgItem(IDC_P1_B2))->GetCheck();
	unsigned char B3 = ((CButton*) GetDlgItem(IDC_P1_B3))->GetCheck();

	unsigned char P1 = ( (B3 << 3) | (B2 << 2) | (B1 << 1) | (B0) ) & 0x0F;
	m_IObuffer.port = P1; 

	DWORD dwBytesSucceed = 0;
	DWORD dwBytesWriteRequest = (sizeof(USB_iobuf)-4);
	DWORD dwBytesReadRequest = (sizeof(USB_iobuf)-4);

	// Init. before write
	dwBytesSucceed = 0;

	// Perform USB Data Transfer
	// Write transfer packet
	status = SI_Write(m_hUSBDevice, &m_IObuffer, dwBytesWriteRequest, &dwBytesSucceed);

	if (dwBytesSucceed != dwBytesWriteRequest || status != SI_SUCCESS)
	{
		m_bWriteError = TRUE;	// Note: Set error flag immediately so that multiple 
								// message boxes do not queue up.
		CString sError;
		sError.Format("Error writing to USB.\nWrote %d of %d bytes.\n\nApplication is aborting.\nReset hardware and try again.", dwBytesSucceed, dwBytesWriteRequest);
		if (AfxMessageBox(sError,MB_OK|MB_ICONEXCLAMATION))
		{
			OnCancel();
			return;
		}
	}

	// Init. before read
	dwBytesSucceed = 0;

	memset(&m_IObuffer, 0, sizeof(USB_iobuf));

	// Read transfer packet
	status = SI_Read(m_hUSBDevice, &m_IObuffer, dwBytesReadRequest, &dwBytesSucceed);

	if (((dwBytesSucceed != dwBytesReadRequest) && (m_bReadError == FALSE)) || status != SI_SUCCESS)
	{
		m_bReadError = TRUE;	// Note: Set error flag immediately so that multiple 
								// message boxes do not queue up.

		CString sError;
		sError.Format("Error reading from USB.\nRead %d of %d bytes.\n\nApplication is aborting.\nReset hardware and try again.", dwBytesSucceed, dwBytesReadRequest);
		if (AfxMessageBox(sError,MB_OK|MB_ICONEXCLAMATION))
		{
			OnCancel();
			return;
		}
	}
		
	// Make updates to dialog display items
	if (m_IObuffer.led1)
		m_ButtonLED1.LedOn();
	else
		m_ButtonLED1.LedOff();

	if (m_IObuffer.led2)
		m_ButtonLED2.LedOn();
	else
		m_ButtonLED2.LedOff();

	unsigned char P0 = m_IObuffer.port & 0x0F;
	((CButton*) GetDlgItem(IDC_P0_B0))->SetCheck(P0 & 0x01);  
	((CButton*) GetDlgItem(IDC_P0_B1))->SetCheck((P0 & 0x02) >> 1);  
	((CButton*) GetDlgItem(IDC_P0_B2))->SetCheck((P0 & 0x04) >> 2);  
	((CButton*) GetDlgItem(IDC_P0_B3))->SetCheck((P0 & 0x08) >> 3);  

	m_Analog_Meter1.UpdateNeedle(m_IObuffer.analog1);
	m_Analog_Meter2.UpdateNeedle(m_IObuffer.analog2);

	CDialog::OnTimer(nIDEvent);
}


BOOL CTestPanelDlg::GetDeviceList()
{
	DWORD	dwNumDevices = 0;
	SI_DEVICE_STRING	devStr;

	SI_STATUS status = SI_GetNumDevices(&dwNumDevices);

	if (status == SI_SUCCESS)
	{
		for (DWORD d = 0; d < dwNumDevices; d++)
		{
			status = SI_GetProductString(d, devStr, SI_RETURN_SERIAL_NUMBER);

			if (status == SI_SUCCESS)
			{
				CString str = devStr;
				m_DeviceList.push_back(str);
			}
		}
	}
	else
	{
		return FALSE;
	}

	return TRUE;
}


BOOL CTestPanelDlg::SelectDevice()
{
	// ask for selection
	CSelectDialog dlg;
	dlg.Initialize(&m_DeviceList);
	if (dlg.DoModal() != IDOK)
		return FALSE;
	m_nDeviceNum = dlg.m_deviceIndex;
	m_sDeviceName = dlg.m_deviceList[dlg.m_deviceIndex];

	return TRUE;
}
