// FileTransferDlg.cpp : implementation file
//

#include "stdafx.h"
#include "FileTransfer.h"
#include "FileTransferDlg.h"
#include "SiUSBXp.h"
#include "dbt.h"
#include "initguid.h"

DEFINE_GUID(GUID_DEVINTERFACE_USBXPRESS, 0x3C5E1462L, 0x5695, 0x4E18, \
			0x87, 0x6B, 0xF3, 0xF3, 0xD0, 0x8A, 0xAF, 0x18);

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
// CFileTransferDlg dialog

CFileTransferDlg::CFileTransferDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CFileTransferDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CFileTransferDlg)
	m_sTXFileName = _T("");
	m_sRXFileName = _T("");
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CFileTransferDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CFileTransferDlg)
	DDX_Text(pDX, IDC_TX_FILE_NAME, m_sTXFileName);
	DDX_Text(pDX, IDC_RX_FILE_NAME, m_sRXFileName);
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CFileTransferDlg, CDialog)
	//{{AFX_MSG_MAP(CFileTransferDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_TRANSFER_DATA, OnTransferData)
	ON_BN_CLICKED(IDC_RECEIVE_DATA, OnReceiveData)
	ON_BN_CLICKED(IDC_BROWSE_TX_FILE, OnBrowseTxFile)
	ON_BN_CLICKED(IDC_BROWSE_RX_FILE, OnBrowseRxFile)
	ON_BN_CLICKED(IDC_UPDATE_DEVICE_LIST, OnUpdateDeviceList)
	ON_WM_DEVICECHANGE()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CFileTransferDlg message handlers

BOOL CFileTransferDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

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
	
	// Register for device notification
	RegisterNotification();

	// Init. member variables
	m_hUSBDevice = INVALID_HANDLE_VALUE;

	// Get devices, init. combo box with their names
	FillDeviceList();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CFileTransferDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CFileTransferDlg::OnPaint() 
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
HCURSOR CFileTransferDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

void CFileTransferDlg::OnTransferData() 
{
	CComboBox*	pDevList	= (CComboBox*)GetDlgItem(IDC_DEVICE_SELECT);

	BeginWaitCursor();

	// Make sure that the device list is valid
	if (pDevList)
	{
		// Open selected device and transfer the file if the selection is valid
		if (pDevList->GetCurSel() >= 0)
		{
			SI_STATUS status = SI_Open(pDevList->GetCurSel(), &m_hUSBDevice);

			if (status == SI_SUCCESS)
			{
				// Write file to device in MAX_PACKET_SIZE_WRITE-byte chunks.
				WriteFileData();

				// Close device.
				SI_Close(m_hUSBDevice);
				m_hUSBDevice = INVALID_HANDLE_VALUE;
			}
		}
	}

	EndWaitCursor();
}

void CFileTransferDlg::RegisterNotification()
{
	// Register device notification
	DEV_BROADCAST_DEVICEINTERFACE devIF = {0};

	devIF.dbcc_size = sizeof(devIF);    
	devIF.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;    
	devIF.dbcc_classguid  = GUID_DEVINTERFACE_USBXPRESS;    

	m_hNotifyDevNode = RegisterDeviceNotification(GetSafeHwnd(), &devIF, DEVICE_NOTIFY_WINDOW_HANDLE);
}

void CFileTransferDlg::UnregisterNotification()
{
	// Unegister device notification
	if(NULL != m_hNotifyDevNode)
	{
		UnregisterDeviceNotification(m_hNotifyDevNode);

		m_hNotifyDevNode = NULL;
	}
}

BOOL CFileTransferDlg::OnDeviceChange(UINT nEventType, DWORD dwData)
{
	switch(nEventType)
	{
	case DBT_DEVICEARRIVAL:        
		// A device has been inserted and is now available.
	
	case DBT_DEVICEREMOVECOMPLETE:        
		// Device has been removed.        
		// Confirm this notify derives from the target device        
		if (DBT_DEVTYP_DEVICEINTERFACE != ((DEV_BROADCAST_HDR *)dwData)->dbch_devicetype)           
			break;        

		if (GUID_DEVINTERFACE_USBXPRESS != ((DEV_BROADCAST_DEVICEINTERFACE *)dwData)->dbcc_classguid)
			break;
		
		if (DBT_DEVICEARRIVAL == nEventType)            
		{
			FillDeviceList();
		}
		else
		{
			SI_Close(m_hUSBDevice);
			m_hUSBDevice = INVALID_HANDLE_VALUE;
			FillDeviceList();
		}
		break;    
	
	default:        
		break;    
	}    
	
	return TRUE;
}

void CFileTransferDlg::OnReceiveData() 
{
	CComboBox*	pDevList	= (CComboBox*)GetDlgItem(IDC_DEVICE_SELECT);

	BeginWaitCursor();

	// Make sure that the device list is valid
	if (pDevList)
	{
		// Open selected device and read the file if the selection is valid
		if (pDevList->GetCurSel() >= 0)
		{
			SI_STATUS status = SI_Open(pDevList->GetCurSel(), &m_hUSBDevice);

			if (status == SI_SUCCESS)
			{
				// Read file data in MAX_PACKET_SIZE_READ-byte chunks and write to temp file.
				// Compare temp. file with original
				ReadFileData();

				// Close device.
				SI_Close(m_hUSBDevice);
				m_hUSBDevice = INVALID_HANDLE_VALUE;
			}
		}
	}

	EndWaitCursor();	
}

void CFileTransferDlg::OnBrowseTxFile() 
{
	CFileDialog fileDlg(TRUE);
	
	// Browse for the TX File
	if (fileDlg.DoModal() == IDOK)
	{
		m_sTXFileName = fileDlg.GetPathName();
		UpdateData(FALSE);
	}
}

void CFileTransferDlg::OnBrowseRxFile() 
{
	CFileDialog fileDlg(TRUE);
	
	// Browse for the RX File
	if (fileDlg.DoModal() == IDOK)
	{
		m_sRXFileName = fileDlg.GetPathName();
		UpdateData(FALSE);
	}
}

void CFileTransferDlg::OnOK() 
{
	// Close handle if open
	if (m_hUSBDevice != INVALID_HANDLE_VALUE)
	{
		// Close the USBXpress device
		SI_Close(m_hUSBDevice);
		m_hUSBDevice = INVALID_HANDLE_VALUE;
	}
	
	// Unregister for device notification
	UnregisterNotification();

	CDialog::OnOK();
}

void CFileTransferDlg::OnUpdateDeviceList() 
{
	FillDeviceList();
}

void CFileTransferDlg::FillDeviceList()
{
	SI_DEVICE_STRING	devStr;
	DWORD				dwNumDevices	= 0;
	CComboBox*			pDevList		= (CComboBox*)GetDlgItem(IDC_DEVICE_SELECT);

	// Make sure that pDevList is not NULL
	if (pDevList)
	{
		// Delete all the strings in the list
		for (int i = 0; i < pDevList->GetCount(); i++)
		{
			pDevList->DeleteString(0);
		}

		// Get the number of USBXpress devices connected
		SI_STATUS status = SI_GetNumDevices(&dwNumDevices);

		if (status == SI_SUCCESS)
		{
			// Go through each device and add their serial numbers to the list
			for (DWORD d = 0; d < dwNumDevices; d++)
			{
				status = SI_GetProductString(d, devStr, SI_RETURN_SERIAL_NUMBER);

				if (status == SI_SUCCESS)
				{
					if (pDevList)
						pDevList->AddString(devStr);
				}
			}
		}

		//Set the current selection to the first item
		pDevList->SetCurSel(0);
	}
}

BOOL CFileTransferDlg::WriteFileData()
{
	BOOL success = TRUE;

	// Make sure a file is specified in the dialog
	if (m_sTXFileName.GetLength() > 0)
	{
		CFile file;

		// Open the file for reading
		if (file.Open(m_sTXFileName, CFile::modeRead | CFile::shareDenyNone))
		{
			// Make sure the file size is 4KB or less
			if (1)//(file.GetLength() && (file.GetLength() <= MAX_PACKET_SIZE_READ))
			{
				DWORD		size			= file.GetLength();
				DWORD		dwBytesWritten	= 0;
				DWORD		dwBytesRead		= 0;
				BYTE		buf[MAX_PACKET_SIZE_WRITE];

				// Set up the write message command with the file size
				buf[0] = FT_WRITE_MSG;
				buf[1] = (BYTE)(size & 0x000000FF);
				buf[2] = (BYTE)((size & 0x0000FF00) >> 8);

				// Send write file size message with a 1 second timeout
				if (DeviceWrite(buf, FT_MSG_SIZE, &dwBytesWritten, 1000))
				{
					if (dwBytesWritten == FT_MSG_SIZE)
					{
						DWORD numPkts		= (size / MAX_PACKET_SIZE_WRITE) + (((size % MAX_PACKET_SIZE_WRITE) > 0)? 1 : 0);
						DWORD numLoops		= (numPkts / MAX_WRITE_PKTS) + (((numPkts % MAX_WRITE_PKTS) > 0)? 1 : 0);
						DWORD counterPkts	= 0;
						DWORD counterLoops	= 0;
						DWORD totalWritten	= 0;

						// Now write data to board
						// After each 512-byte packet, the device will send an 0xFF ACK signal
						while ((counterLoops < numLoops) && success)
						{
							int	i = 0;

							while ((i < MAX_WRITE_PKTS) && (counterPkts < numPkts) && success)
							{
								DWORD dwWriteLength	= 0;

								// Determine if the size is the maximum, or a partial packet
								if ((size - totalWritten) < MAX_PACKET_SIZE_WRITE)
								{
									dwWriteLength = size - totalWritten;
								}
								else
								{
									dwWriteLength = MAX_PACKET_SIZE_WRITE;
								}

								// Read in the data for the packet from the file
								memset(buf, 0, dwWriteLength);
								file.Read(buf, dwWriteLength);
								dwBytesWritten = 0;

								//Write to the device with a 1 second timeout
								success = DeviceWrite(buf, dwWriteLength, &dwBytesWritten, 1000);
								totalWritten += dwWriteLength;
								counterPkts++;
								i++;
							}

							if (success)
							{
								memset(buf, 0, 1);

								// Check for ACK packet after writing 512 bytes or after last packet
								// with a 3 second timeout
								while ((buf[0] != 0xFF) && success)
								{
									success = DeviceRead(buf, 1, &dwBytesRead, 3000);
								}
							}

							counterLoops++;
						}
						
						if (!success)
						{
							AfxMessageBox("Target device failure while sending file data.\nCheck file size.");
							success = FALSE;
						}
					}
					else
					{
						AfxMessageBox("Incomplete write file size message sent to device.");
						success = FALSE;
					}
				}
				else
				{
					AfxMessageBox("Target device failure while sending file size information.");
					success = FALSE;
				}
			}
			else
			{
				AfxMessageBox("Limit for file size is 4KB");
				success = FALSE;
			}

			file.Close();
		}
		else
		{
			CString err;
			err.Format("Failed opening file:\n%s", m_sTXFileName);
			AfxMessageBox(err);
			success = FALSE;
		}
	}
	else
	{
		AfxMessageBox("Error:  No file selected.");
		success = FALSE;
	}

	return success;
}

BOOL CFileTransferDlg::ReadFileData()
{
	BOOL success = TRUE;
    DWORD	length;
	if ((length = m_sRXFileName.GetLength()) > 0)
	{
		CFile	file;

		// Open the temporary file for writing
		if (file.Open(m_sRXFileName, CFile::modeWrite | CFile::modeCreate | CFile::shareDenyNone))
		{
			DWORD		dwBytesRead		= 0;
			DWORD		dwBytesWritten	= 0;
			BYTE		buf[MAX_PACKET_SIZE_READ];
			BYTE		msg[FT_MSG_SIZE];

			// Setup the read command buffer
			msg[0] = FT_READ_MSG;
			msg[1] = (BYTE)0xFF;
			msg[2] = (BYTE)0xFF;

			// Send the read command buffer to the device with a 1 second timeout
			if (DeviceWrite(msg, FT_MSG_SIZE, &dwBytesWritten, 1000))
			{
				DWORD size			= 0;
				DWORD counterPkts	= 0;
				DWORD numPkts		= 0;
				DWORD totalRead		= 0;

				// Reset the message buffer
				memset(msg, 0, FT_MSG_SIZE);

				// Read the response containing the size with a 3 second timeout
				if (DeviceRead(buf, FT_MSG_SIZE, &dwBytesRead, 3000))
				{
					size	= (buf[1] & 0x000000FF) | ((buf[2] << 8) & 0x0000FF00);
					numPkts	= (size/MAX_PACKET_SIZE_READ) + (((size % MAX_PACKET_SIZE_READ) > 0)? 1 : 0);

					// Now read data from board
					while ((counterPkts < numPkts) && success)
					{
						DWORD dwReadLength = 0;
						dwBytesRead = 0;

						// Determine if the size is the max or a partial packet
						if ((size - totalRead) < MAX_PACKET_SIZE_READ)
						{
							dwReadLength = size - totalRead;
						}
						else
						{
							dwReadLength = MAX_PACKET_SIZE_READ;
						}
						
						memset(buf, 0, dwReadLength);
						
						//Read the packet from the device with a 3 second timeout
						if (DeviceRead(buf, dwReadLength, &dwBytesRead, 3000))
						{
							totalRead += dwBytesRead;
							file.Write(buf, dwBytesRead);
							if (dwBytesRead == dwReadLength)
							{
								counterPkts++;
							}
						}
						else
						{
							AfxMessageBox("Failed reading file packet from target device.");
							success = FALSE;
						}


					}
					
				}
				else
				{
					AfxMessageBox("Failed reading file size message from target device.");
					success = FALSE;
				}
			}
			else
			{
				AfxMessageBox("Failed sending read file message to target device.");
				success = FALSE;
			}

			if (file.GetLength() == 0)
			{
				CString err;
				err.Format("File has 0 length:\n%s", m_sRXFileName);
				AfxMessageBox(err);
			}

			file.Close();
		}
		else
		{
			CString err;
			err.Format("Failed opening file:\n%s", m_sRXFileName);
			AfxMessageBox(err);
			success = FALSE;
		}
	}
	else
	{
		AfxMessageBox("Error:  No file selected.");
		success = FALSE;
	}

	return success;
}

BOOL CFileTransferDlg::DeviceRead(BYTE* buffer, DWORD dwSize, DWORD* lpdwBytesRead, DWORD dwTimeout)
{
	DWORD		tmpReadTO, tmpWriteTO;
	SI_STATUS	status			= SI_SUCCESS;

	// Save timeout values.
	SI_GetTimeouts(&tmpReadTO, &tmpWriteTO);

	// Set a timeout for the read
	SI_SetTimeouts(dwTimeout, 0);

	// Read the data
	status = SI_Read(m_hUSBDevice, buffer, dwSize, lpdwBytesRead);
	
	// Restore timeouts
	SI_SetTimeouts(tmpReadTO, tmpWriteTO);

	return (status == SI_SUCCESS);
}

BOOL CFileTransferDlg::DeviceWrite(BYTE* buffer, DWORD dwSize, DWORD* lpdwBytesWritten, DWORD dwTimeout)
{
	DWORD		tmpReadTO, tmpWriteTO;
	SI_STATUS	status	= SI_SUCCESS;

	// Save timeout values.
	SI_GetTimeouts(&tmpReadTO, &tmpWriteTO);

	// Set a timeout for the write
	SI_SetTimeouts(0, dwTimeout);

	// Write the data
	status = SI_Write(m_hUSBDevice, buffer, dwSize, lpdwBytesWritten);

	// Restore timeouts
	SI_SetTimeouts(tmpReadTO, tmpWriteTO);

	return (status == SI_SUCCESS);
}
