// SelectDialog.cpp : implementation file
//

#include "stdafx.h"
#include "TestPanel.h"
#include "SelectDialog.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CSelectDialog dialog


CSelectDialog::CSelectDialog(CWnd* pParent /*=NULL*/)
	: CDialog(CSelectDialog::IDD, pParent)
{
	//{{AFX_DATA_INIT(CSelectDialog)
	m_deviceIndex = -1;
	//}}AFX_DATA_INIT
}


void CSelectDialog::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CSelectDialog)
	DDX_Control(pDX, IDC_DEVICELIST, m_ctlDevicelist);
	DDX_CBIndex(pDX, IDC_DEVICELIST, m_deviceIndex);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CSelectDialog, CDialog)
	//{{AFX_MSG_MAP(CSelectDialog)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CSelectDialog message handlers

void CSelectDialog::Initialize(DeviceList* devList) 
{
	//CDialog::OnInitDialog();
	m_deviceList = *devList;
}

BOOL CSelectDialog::OnInitDialog()
{
	CDialog::OnInitDialog();

	for (int i = 0; i < m_deviceList.size(); i++)
	{
		m_ctlDevicelist.AddString(m_deviceList[i]);
	}
	m_ctlDevicelist.SetCurSel(i - 1);

	return TRUE;
	
}
