// TestPanelDlg.h : header file
//

#if !defined(AFX_TESTPANELDLG_H__A6A66517_BBBE_4B75_BA11_67CFC720468B__INCLUDED_)
#define AFX_TESTPANELDLG_H__A6A66517_BBBE_4B75_BA11_67CFC720468B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// Custom Controls
#include "DynamicLED.h"
#include "3DMeterCtrl.h"
#include <vector>

//
// buffer to transmit/receive USB data
//
struct USB_iobuf {     
  unsigned char led1, led2;
  unsigned char	port;
  unsigned char analog1, analog2;
  unsigned char not_used[3];
  unsigned char number_of_interrupts[4];
};

typedef	std::vector<CString>	DeviceList;

/////////////////////////////////////////////////////////////////////////////
// CTestPanelDlg dialog

class CTestPanelDlg : public CDialog
{
// Construction
public:
	CTestPanelDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	//{{AFX_DATA(CTestPanelDlg)
	enum { IDD = IDD_TESTPANEL_DIALOG };
	C3DMeterCtrl	m_Analog_Meter1;
	C3DMeterCtrl	m_Analog_Meter2;
	CDynamicLED		m_ButtonLED1;
	CDynamicLED		m_ButtonLED2;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CTestPanelDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation

protected:
	HICON		m_hIcon;
	HANDLE		m_hUSBDevice;

	BOOL		m_bReadError;
	BOOL		m_bWriteError;

	// USB data buffer
	USB_iobuf	m_IObuffer;

	DeviceList	m_DeviceList;
	int			m_nDeviceNum;
	CString		m_sDeviceName;

	BOOL	GetDeviceList();
	BOOL	SelectDevice();

	// Generated message map functions
	//{{AFX_MSG(CTestPanelDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	virtual void OnOKExit();
	virtual void OnCancel();
	afx_msg void OnTimer(UINT nIDEvent);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TESTPANELDLG_H__A6A66517_BBBE_4B75_BA11_67CFC720468B__INCLUDED_)
