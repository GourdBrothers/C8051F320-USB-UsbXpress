// FileTransferDlg.h : header file
//

#if !defined(AFX_FILETRANSFERDLG_H__41AB8D1D_67C8_4FBA_880E_B148DBD01961__INCLUDED_)
#define AFX_FILETRANSFERDLG_H__41AB8D1D_67C8_4FBA_880E_B148DBD01961__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// The size of the FLASH pages on the C8051F320 is 512 bytes
// For this reason, the application will only write 512 bytes at a time,
// but it will read 4096 bytes at a time
#define MAX_PACKET_SIZE_WRITE	512
#define MAX_PACKET_SIZE_READ	4096	

#define MAX_WRITE_PKTS		0x01

#define FT_READ_MSG			0x00
#define FT_WRITE_MSG		0x01
#define FT_READ_ACK			0x02
#define FT_MSG_SIZE			0x03

/////////////////////////////////////////////////////////////////////////////
// CFileTransferDlg dialog

class CFileTransferDlg : public CDialog
{
// Construction
public:
	CFileTransferDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	//{{AFX_DATA(CFileTransferDlg)
	enum { IDD = IDD_FILETRANSFER_DIALOG };
	CString	m_sTXFileName;
	CString	m_sRXFileName;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CFileTransferDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CFileTransferDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnTransferData();
	afx_msg void OnReceiveData();
	afx_msg void OnBrowseTxFile();
	afx_msg void OnBrowseRxFile();
	virtual void OnOK();
	afx_msg void OnUpdateDeviceList();
	afx_msg BOOL OnDeviceChange(UINT nEventType, DWORD dwData); 
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

private:
	void FillDeviceList();
	BOOL WriteFileData();
	BOOL ReadFileData();
	BOOL DeviceRead(BYTE* buffer, DWORD dwSize, DWORD* lpdwBytesRead, DWORD dwTimeout = INFINITE);
	BOOL DeviceWrite(BYTE* buffer, DWORD dwSize, DWORD* lpdwBytesWritten, DWORD dwTimeout = INFINITE);

	void RegisterNotification();
	void UnregisterNotification();

private:
	HANDLE m_hUSBDevice;

	HDEVNOTIFY m_hNotifyDevNode;
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_FILETRANSFERDLG_H__41AB8D1D_67C8_4FBA_880E_B148DBD01961__INCLUDED_)
