// FileTransfer.h : main header file for the FILETRANSFER application
//

#if !defined(AFX_FILETRANSFER_H__E878A719_4D2D_4CC0_A189_1B6841B1D932__INCLUDED_)
#define AFX_FILETRANSFER_H__E878A719_4D2D_4CC0_A189_1B6841B1D932__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols

/////////////////////////////////////////////////////////////////////////////
// CFileTransferApp:
// See FileTransfer.cpp for the implementation of this class
//

class CFileTransferApp : public CWinApp
{
public:
	CFileTransferApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CFileTransferApp)
	public:
	virtual BOOL InitInstance();
	//}}AFX_VIRTUAL

// Implementation

	//{{AFX_MSG(CFileTransferApp)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_FILETRANSFER_H__E878A719_4D2D_4CC0_A189_1B6841B1D932__INCLUDED_)
