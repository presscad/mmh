#include "stdafx.h"
#include ".\tdialog.h"

TDialog::TDialog(UINT nIDTemplate, CWnd* pParent , int nIDDlgItem)
	: CDialog(nIDTemplate, pParent)
{
	m_nIDD=nIDTemplate;
	m_bCheckShow=false;

	m_pParent=pParent;
	m_nIDdlgItem=nIDDlgItem;
}

TDialog::~TDialog(void)
{
}

void TDialog::create()
{
	__super::Create(m_nIDD, m_pParent);
	CRect rect;
	m_pParent->GetDlgItem(m_nIDdlgItem)->GetWindowRect(rect);
	m_pParent->GetDlgItem(m_nIDdlgItem)->ShowWindow(SW_HIDE);
	m_pParent->ScreenToClient(rect);
	MoveWindow(rect);
}

bool TDialog::show()
{
	if(!m_bCheckShow)
	{
		ShowWindow(SW_SHOW);
		GetParent()->Invalidate();
		m_bCheckShow=true;
		return true;
	}
	return false;
}

bool TDialog::hide()
{
	if(m_bCheckShow)
	{
		ShowWindow(SW_HIDE);
		m_bCheckShow=false;
		return true;
	}
	return false;
}

int TDialog::addChild(TDialog* pChild)
{
	pChild->create();
	m_aChildren.pushBack(pChild);
	return m_aChildren.size()-1;
}

void TDialog::embedDlg(int nChildIndex)
{
	for(int i=0; i<m_aChildren.size(); i++)
		m_aChildren[i].hide();

	m_aChildren[nChildIndex].show();
}