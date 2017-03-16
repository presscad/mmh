#pragma once
#include "afxwin.h"
#include "trefarray.h"
class TDialog :
	public CDialog
{
public:
	
	// show하기전 반드시 create해주어야 한다.
	TDialog(UINT nIDTemplate, CWnd* pParent, int nIDDlgItem); //!< nIDDlgItem은 IDC_BUTTON_DRAW_MODLG 처럼 button의 id를 주면 그 자리에 dialog가 생긴다. 아직 보이지는 않는다.
	virtual ~TDialog(void);

	UINT m_nIDD;

	virtual void create();

	virtual bool show();
	virtual bool hide();

	
	/**
	 * 자신의 특정 item위치에 childDlg Create밑 삽입.
	 * 자신이 create된 후 호출할수 있다. 가장 좋은 방법은 create를 상속해서 __super::create수행되 addchild하는것이다.
	 * \param pChild new만 된 TDialog instance
	 * \param nIDDlgItem create될 위치를 나타내는 iditem
	 * \return nChildIndex (embedDlg할때 사용됨)
	 */
	int addChild(TDialog* pChild);	
	void embedDlg(int nChildIndex);
	int numChildDlg()					{ return m_aChildren.size();}
	TDialog& childDlg(int nChildIndex)	{ return m_aChildren[nChildIndex];}
	
protected:	

	CWnd* m_pParent;
	int m_nIDdlgItem;
	bool m_bCheckShow;
	TRefArray<TDialog> m_aChildren;
};
