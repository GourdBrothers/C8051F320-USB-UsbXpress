#if !defined(AFX_SELECTDIALOG_H__00A4E40B_242F_4E2D_9912_2D1F4DD90E53__INCLUDED_)
#define AFX_SELECTDIALOG_H__00A4E40B_242F_4E2D_9912_2D1F4DD90E53__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// SelectDialog.h : header file
//

#include "TestPanelDlg.h"

/////////////////////////////////////////////////////////////////////////////
// CSelectDialog dialog

class CSelectDialog : public CDialog
{
// Construction
public:
	CSelectDialog(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CSelectDialog)
	enum { IDD = IDD_SELECT };
	CComboBox	m_ctlDevicelist;
	int		m_deviceIndex;
	//}}AFX_DATA
	DeviceList m_deviceList;
	void Initialize(DeviceList* devList);
	

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CSelectDialog)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
	
	
protected:

	// Generated message map functions
	//{{AFX_MSG(CSelectDialog)
	virtual BOOL OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SELECTDIALOG_H__00A4E40B_242F_4E2D_9912_2D1F4DD90E53__INCLUDED_)
