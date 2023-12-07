
// demoDlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "demo.h"
#include "demoDlg.h"
#include "afxdialogex.h"

#include "image.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
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


// CdemoDlg 对话框



CdemoDlg::CdemoDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_DEMO_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CdemoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CdemoDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CdemoDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDOK, &CdemoDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CdemoDlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON2, &CdemoDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CdemoDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &CdemoDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &CdemoDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON6, &CdemoDlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON7, &CdemoDlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON8, &CdemoDlg::OnBnClickedButton8)
	ON_BN_CLICKED(IDC_BUTTON9, &CdemoDlg::OnBnClickedButton9)
	ON_BN_CLICKED(IDC_BUTTON10, &CdemoDlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_BUTTON11, &CdemoDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON12, &CdemoDlg::OnBnClickedButton12)
	ON_BN_CLICKED(IDC_BUTTON13, &CdemoDlg::OnBnClickedButton13)
	ON_BN_CLICKED(IDC_BUTTON14, &CdemoDlg::OnBnClickedButton14)
	ON_BN_CLICKED(IDC_BUTTON15, &CdemoDlg::OnBnClickedButton15)
	ON_BN_CLICKED(IDC_BUTTON16, &CdemoDlg::OnBnClickedButton16)
	ON_BN_CLICKED(IDC_BUTTON17, &CdemoDlg::OnBnClickedButton17)
	ON_BN_CLICKED(IDC_BUTTON18, &CdemoDlg::OnBnClickedButton18)
	ON_BN_CLICKED(IDC_BUTTON19, &CdemoDlg::OnBnClickedButton19)
	ON_BN_CLICKED(IDC_BUTTON20, &CdemoDlg::OnBnClickedButton20)
	ON_BN_CLICKED(IDC_BUTTON21, &CdemoDlg::OnBnClickedButton21)
	ON_BN_CLICKED(IDC_BUTTON22, &CdemoDlg::OnBnClickedButton22)
	ON_BN_CLICKED(IDC_BUTTON23, &CdemoDlg::OnBnClickedButton23)
	ON_BN_CLICKED(IDC_BUTTON24, &CdemoDlg::OnBnClickedButton24)
	ON_BN_CLICKED(IDC_BUTTON25, &CdemoDlg::OnBnClickedButton25)
	ON_BN_CLICKED(IDC_BUTTON26, &CdemoDlg::OnBnClickedButton26)
	ON_BN_CLICKED(IDC_BUTTON27, &CdemoDlg::OnBnClickedButton27)
	ON_BN_CLICKED(IDC_BUTTON28, &CdemoDlg::OnBnClickedButton28)
END_MESSAGE_MAP()


// CdemoDlg 消息处理程序

BOOL CdemoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
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

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	AllocConsole();

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CdemoDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CdemoDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CdemoDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CdemoDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	//MessageBox(L"Hello World");
	ImageShow();
}


void CdemoDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	//CDialogEx::OnOK();
	destroyAllWindows();
}


void CdemoDlg::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnCancel();
}


void CdemoDlg::OnBnClickedButton2()
{
	// TODO: 在此添加控件通知处理程序代码
	ImageGray(".\\data\\lena.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton3()
{
	// TODO: 在此添加控件通知处理程序代码
	DrawHist(".\\data\\lena.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton4()
{
	// TODO: 在此添加控件通知处理程序代码
	HistEqualize(".\\data\\lena.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton5()
{
	// TODO: 在此添加控件通知处理程序代码
	ImageGradSharpen(".\\data\\lena.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton6()
{
	// TODO: 在此添加控件通知处理程序代码
	ImageLaplaceSharpen(".\\data\\lena.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton7()
{
	// TODO: 在此添加控件通知处理程序代码
	RobertsEdgeDetection(".\\data\\lena.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton8()
{
	// TODO: 在此添加控件通知处理程序代码
	Mat src = imread(".\\data\\lena.jpg", IMREAD_GRAYSCALE);
	SobelEdgeDetection(src);
	waitKey();
}


void CdemoDlg::OnBnClickedButton9()
{
	// TODO: 在此添加控件通知处理程序代码
	LaplaceEdgeDetection(".\\data\\lena.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton10()
{
	// TODO: 在此添加控件通知处理程序代码
	CannyEdgeDetection(".\\data\\lena.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton11()
{
	// TODO: 在此添加控件通知处理程序代码
	Mat src, dst;
	src = imread(".\\data\\lena.jpg", IMREAD_COLOR);
	imshow("src", src);
	AddSalt(src, 3000);
	imshow("saltImage", src);
	imwrite(".\\data\\lena_salt.jpg", src);
	waitKey();
}


void CdemoDlg::OnBnClickedButton12()
{
	// TODO: 在此添加控件通知处理程序代码
	Mat src, dst;
	src = imread(".\\data\\lena.jpg", IMREAD_COLOR);
	dst = addGaussianNoise(src);
	imshow("src", src);
	imshow("gaussianImage", dst);
	imwrite(".\\data\\lena_gaussian.jpg", dst);

}


void CdemoDlg::OnBnClickedButton13()
{
	// TODO: 在此添加控件通知处理程序代码
	Mat src, blurfilter, medianBlurfilter, sideblurfilter, gaussianfilter;
	src = imread(".\\data\\lena_salt.jpg", IMREAD_COLOR);
	//src = imread(".\\data\\lena_gaussian.jpg", IMREAD_COLOR);
	imshow("saltImage", src);

	Filter filter(src);

	// 均值滤波
	blurfilter = filter.ImageBlurFilter();
	imshow("blur", blurfilter);

	// 中值滤波
	medianBlurfilter = filter.ImageMedianBlurFilter();
	imshow("medianBlur", medianBlurfilter);

	// 均值滤波+边窗滤波
	sideblurfilter = filter.ImageSideBlurFilter();
	imshow("side_blur", sideblurfilter);

	// 高斯滤波
	gaussianfilter = filter.ImageGaussianFilter();
	imshow("gaussian", gaussianfilter);

	waitKey();
}


void CdemoDlg::OnBnClickedButton14()
{
	// TODO: 在此添加控件通知处理程序代码
	Mat src, erodeImage, dilateImage;
	src = imread(".\\data\\source.jpg");
	Mat struct_element;
	struct_element = getStructuringElement(MORPH_RECT, Size(13, 13));
	erode(src, erodeImage, struct_element);
	imshow("src", src);
	imshow("erodeImage", erodeImage);
	dilate(erodeImage, dilateImage, struct_element);
	imshow("dilateImage", dilateImage);
	waitKey();
}


void CdemoDlg::OnBnClickedButton15()
{
	// TODO: 在此添加控件通知处理程序代码
	SignalCamera();
}


void CdemoDlg::OnBnClickedButton16()
{
	// TODO: 在此添加控件通知处理程序代码
	DoubleCamera();
}


void CdemoDlg::OnBnClickedButton17()
{
	// TODO: 在此添加控件通知处理程序代码
	ImageWarpAffine(".\\data\\lena_salt.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton18()
{
	// TODO: 在此添加控件通知处理程序代码
	Mat src;
	src = imread(".\\data\\lena_salt.jpg", IMREAD_COLOR);
	Point2f srcPoint[4], dstPoint[4];
	srcPoint[0] = Point2f(0, 0);
	srcPoint[1] = Point2f(src.cols - 1, 0);
	srcPoint[2] = Point2f(0, src.rows - 1);
	srcPoint[3] = Point2f(src.cols - 1, src.rows - 1);
	dstPoint[0] = Point2f(src.cols * 0.05, src.rows * 0.33);
	dstPoint[1] = Point2f(src.cols * 0.9, src.rows * 0.25);
	dstPoint[2] = Point2f(src.cols * 0.2, src.rows * 0.7);
	dstPoint[3] = Point2f(src.cols * 0.8, src.rows * 0.9);
	ImageWrapPerspective(src, srcPoint, dstPoint, src.size());
	waitKey();
}


void CdemoDlg::OnBnClickedButton19()
{
	// TODO: 在此添加控件通知处理程序代码
	StereoMatch();
	waitKey();
}


void CdemoDlg::OnBnClickedButton20()
{
	// TODO: 在此添加控件通知处理程序代码
	ThresholdSeparation fixed;
	fixed.FixedThreshold(".//data//lena.jpg", 100);
	waitKey();
}


void CdemoDlg::OnBnClickedButton21()
{
	// TODO: 在此添加控件通知处理程序代码
	ThresholdSeparation fixed;
	fixed.OTSUThreshold(".//data//lena.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton22()
{
	// TODO: 在此添加控件通知处理程序代码
	ThresholdSeparation fixed;
	Mat src = imread(".//data//lena.jpg", IMREAD_GRAYSCALE);
	fixed.KitlteThreshold(src);
	waitKey();
}


void CdemoDlg::OnBnClickedButton23()
{
	// TODO: 在此添加控件通知处理程序代码
	//DrivewaySplitAndMix(".\\data\\test_images\\straight_lines1.jpg");
	Mat src_BGR;
    src_BGR = imread(".\\data\\test_images\\straight_lines1.jpg", IMREAD_COLOR);
	DrivewaySplitAndMix(src_BGR);
	//DrivewaySplitAndMix(".\\data\\test_images\\test3.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton24()
{
	// TODO: 在此添加控件通知处理程序代码
	//Process2(".\\data\\test_images\\straight_lines1.jpg", ".\\data\\test_images\\preprocess.jpg");
	Mat src2, src;
	String filename1, filename2;
	filename1 = ".\\data\\test_images\\straight_lines1.jpg";
	filename2 = ".\\data\\test_images\\preprocess.jpg";
	src = imread(filename2, IMREAD_GRAYSCALE);
	src2 = imread(filename1);
	Process2(src2, src);
	waitKey();
}


void CdemoDlg::OnBnClickedButton25()
{
	// TODO: 在此添加控件通知处理程序代码
	HoughLineDetection(".\\data\\test_images\\straight_lines1.jpg");
	//HoughLineDetection(".\\data\\test_images\\straight_lines2.jpg");
	waitKey();
}


void CdemoDlg::OnBnClickedButton26()
{
	// TODO: 在此添加控件通知处理程序代码
	//Brisk_BlemishDetection(".//data//1.1.jpg", ".//data//1.2.jpg");
	ORB_BlemishDetection(".//data//1.1.jpg", ".//data//1.2.jpg");
}


void CdemoDlg::OnBnClickedButton27()
{
	// TODO: 在此添加控件通知处理程序代码
	CarLineVideo(".\\data\\test_images\\project_video.mp4");
}


void CdemoDlg::OnBnClickedButton28()
{
	// TODO: 在此添加控件通知处理程序代码
	TargetTracking();
}
