//------------------------------------------------------------------------------
// <copyright file="DepthBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#define NOMINMAX

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "DepthBasics.h"

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CDepthBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CDepthBasics::CDepthBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_bSaveScreenshot(false),
    m_pKinectSensor(NULL),
    m_pDepthFrameReader(NULL),
    m_pD2DFactory(NULL),
    m_pDrawDepth(NULL),
    m_pDrawColor(NULL),
    m_pDepthRGBX(NULL),
    m_pColorRGBX(NULL),
    m_pMultiSourceReader(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }
    this->localMutex = new igtl::SimpleMutexLock;
    // create heap storage for depth pixel data in RGBX format
    m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
    // create heap storage for the coorinate mapping from color to depth
    m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];
    memset(&info, 0, sizeof(SFrameBSInfo));
    memset(&pic, 0, sizeof(SSourcePicture));
    memset(&infoIndex, 0, sizeof(SFrameBSInfo));
    memset(&picIndex, 0, sizeof(SSourcePicture));
    memset(&infoColor, 0, sizeof(SFrameBSInfo));
    memset(&picColor, 0, sizeof(SSourcePicture));
    int frameSize;
    if (useDemux)
    {
      if (DemuxMethod == 1)
      {
        frameSize = 4 * cDepthWidth* cDepthHeight * 3 / 2;
        pic.iPicWidth = cDepthWidth * 2;
        pic.iPicHeight = cDepthHeight * 2;
      }
      else if (DemuxMethod == 2)
      {
        frameSize = cDepthWidth* cDepthHeight * 2;
        pic.iPicWidth = cDepthWidth;
        pic.iPicHeight = cDepthHeight;
      }
      else if (DemuxMethod == 3)
      {
        frameSize = cDepthWidth* cDepthHeight * 3 / 2;
        pic.iPicWidth = cDepthWidth;
        pic.iPicHeight = cDepthHeight;
        picIndex.iPicWidth = cDepthWidth;
        picIndex.iPicHeight = cDepthHeight;
      }
    }
    else
    {
      frameSize = cDepthWidth* cDepthHeight * 3 / 2;
      pic.iPicWidth = cDepthWidth;
      pic.iPicHeight = cDepthHeight;
    }
    m_pDepthYUV420.SetLength(frameSize);
    pic.iColorFormat = videoFormatI420;
    pic.iStride[0] = pic.iPicWidth;
    pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
    pic.pData[0] = m_pDepthYUV420.data();
    pic.pData[1] = pic.pData[0] + pic.iPicWidth * pic.iPicHeight;
    pic.pData[2] = pic.pData[1] + (pic.iPicHeight * pic.iPicHeight >> 2);
    m_pDepthIndexYUV420.SetLength(frameSize);
    picIndex.iColorFormat = videoFormatI420;
    picIndex.iStride[0] = picIndex.iPicWidth;
    picIndex.iStride[1] = picIndex.iStride[2] = picIndex.iPicWidth >> 1;
    picIndex.pData[0] = m_pDepthIndexYUV420.data();
    picIndex.pData[1] = picIndex.pData[0] + picIndex.iPicWidth * picIndex.iPicHeight;
    pic.pData[2] = picIndex.pData[1] + (picIndex.iPicHeight * picIndex.iPicHeight >> 2);

    picColor.iPicWidth = cDepthWidth;
    picColor.iPicHeight = cDepthHeight;
    if (useCompressForRGB)
    {
      m_pColorYUV420.SetLength(cDepthWidth* cDepthHeight * 3 / 2);
    }
    else
    {
      m_pColorYUV420.SetLength(cDepthWidth* cDepthHeight * 3);
    }
    picColor.iColorFormat = videoFormatI420;
    picColor.iStride[0] = picColor.iPicWidth;
    picColor.iStride[1] = picColor.iStride[2] = picColor.iPicWidth >> 1;
    picColor.pData[0] = m_pColorYUV420.data();
    picColor.pData[1] = picColor.pData[0] + picColor.iPicWidth * picColor.iPicHeight;
    picColor.pData[2] = picColor.pData[1] + (picColor.iPicHeight * picColor.iPicHeight >> 2);
    
    // Initial the openigtlink server
    threaderServer = igtl::MultiThreader::New();
    glockServer = igtl::MutexLock::New();
    td_Server.portNum = 18944;
    td_Server.stop = 1;
    td_Server.pic = pic;
    td_Server.info = info;
    td_Server.pic_Index = picIndex;
    td_Server.info_Index = infoIndex;
    td_Server.pic_Color = picColor;
    td_Server.info_Color = infoColor;
    td_Server.transmissionFinished = true;
    td_Server.conditionVar = igtl::ConditionVariable::New();
    threaderServer->SpawnThread((igtl::ThreadFunctionType) &DepthImageServer::ServerControl, &td_Server);
    
}
  

/// <summary>
/// Destructor
/// </summary>
CDepthBasics::~CDepthBasics()
{
    // clean up Direct2D renderer
    if (m_pDrawDepth)
    {
        delete m_pDrawDepth;
        m_pDrawDepth = NULL;
    }

    if (m_pDrawColor)
    {
      delete m_pDrawColor;
      m_pDrawColor = NULL;
    }


    if (m_pDepthRGBX)
    {
      delete[] m_pDepthRGBX;
      m_pDepthRGBX = NULL;
    }

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with depth frame reader
    SafeRelease(m_pDepthFrameReader);

    // done with depth frame reader
    SafeRelease(m_pColorFrameReader);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CDepthBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"DepthBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CDepthBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);
    // Main message loop


    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CDepthBasics::Update()
{
    if (!m_pDepthFrameReader)
    {
        return;
    }

    IDepthFrame* pDepthFrame = NULL;
    IColorFrame* pColorFrame = NULL;
    HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        USHORT nDepthMinReliableDistance = 0;
        USHORT nDepthMaxDistance = 0;
        UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;

        IFrameDescription* pFrameDescriptionColor = NULL;
        int nWidthColor = 0;
        int nHeightColor = 0;
        UINT nBufferSizeColor = 0;
        RGBQUAD *pBufferColor = NULL;

        hr = pDepthFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
        }

        if (SUCCEEDED(hr))
        {
			    // In order to see the full range of depth (including the less reliable far field depth)
			    // we are setting nDepthMaxDistance to the extreme potential depth threshold
			    //nDepthMaxDistance = USHRT_MAX;

			    // Note:  If you wish to filter by reliable depth distance, uncomment the following line.
          hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
        }

        if (SUCCEEDED(hr))
        {
          //nDepthMaxDistance = nDepthMinReliableDistance + 255;
          ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
        }
        SafeRelease(pFrameDescription);

        hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
        if (SUCCEEDED(hr))
        {
          hr = pColorFrame->get_FrameDescription(&pFrameDescriptionColor);
        }

        if (SUCCEEDED(hr))
        {
          hr = pFrameDescriptionColor->get_Width(&nWidthColor);
        }

        if (SUCCEEDED(hr))
        {
          hr = pFrameDescriptionColor->get_Height(&nHeightColor);
        }

        ColorImageFormat imageFormat = ColorImageFormat_None;
        if (SUCCEEDED(hr))
        {
          hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }

        if (SUCCEEDED(hr))
        {
          if (imageFormat == ColorImageFormat_Bgra)
          {
            hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSizeColor, reinterpret_cast<BYTE**>(&pBufferColor));
          }
          else if (m_pColorRGBX)
          {
            pBufferColor = m_pColorRGBX;
            nBufferSizeColor = cColorWidth * cColorHeight * sizeof(RGBQUAD);
            hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSizeColor, reinterpret_cast<BYTE*>(pBufferColor), ColorImageFormat_Bgra);
          }
          else
          {
            hr = E_FAIL;
          }
        }
        if (SUCCEEDED(hr))
        {
          ProcessColor(nTime, pBuffer, pBufferColor,nWidth,nHeight, nWidthColor, nHeightColor);
        }
        if (Synchonize)
        {
          this->localMutex->Lock();
          this->td_Server.transmissionFinished = false;
          while (!this->td_Server.transmissionFinished)
            this->td_Server.conditionVar->Wait(this->localMutex);
          this->localMutex->Unlock();
        }
        SafeRelease(pFrameDescriptionColor);
    }
    SafeRelease(pDepthFrame);
    SafeRelease(pColorFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CDepthBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CDepthBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CDepthBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>

LRESULT CALLBACK CDepthBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawDepth = new ImageRenderer();
            m_pDrawColor = new ImageRenderer();
            HRESULT hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cDepthWidth, cDepthHeight, cDepthWidth * sizeof(RGBQUAD)); 
            hr = m_pDrawColor->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOCOLORVIEW), m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
            LPTSTR lpString = L"18944";
            SetDlgItemText(m_hWnd, IDC_EDIT1, lpString);
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the screenshot control and a button clicked event, save a screenshot next frame 
            if (IDC_BUTTON_SCREENSHOT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
            {
                m_bSaveScreenshot = true;
            }
			      // If it was for the check box, start the server
			      if (IDC_CHECK3 == LOWORD(wParam))
			      {
              LPTSTR lpString= new TCHAR[5];
              GetDlgItemText(m_hWnd, IDC_EDIT1, lpString, 5);
              td_Server.portNum = atoi((const char*)lpString);
              td_Server.stop = (bool) HIWORD(wParam);
			      }
            break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CDepthBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = NULL;
        IColorFrameSource* pColorFrameSource = NULL;
        //m_pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes_Color | FrameSourceTypes_Depth, &m_pMultiSourceReader);
        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
        }

        if (SUCCEEDED(hr))
        {
          m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
        }


        SafeRelease(pDepthFrameSource);
        SafeRelease(pColorFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// <param name="nMinDepth">minimum reliable depth</param>
/// <param name="nMaxDepth">maximum reliable depth</param>
/// </summary>
void CDepthBasics::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
    if (m_hWnd)
    {
        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }

    // Make sure we've received valid data
    if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
    {
        RGBQUAD* pRGBX = m_pDepthRGBX;

        // end pixel is start + width*height - 1
        const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
        uint8_t* pDepth = m_pDepthYUV420.data();
        uint8_t* pDepthIndex = m_pDepthIndexYUV420.data();
        for(int j = 0; j < nHeight; ++j)
        { 
          for (int i = 0; i<nWidth; ++i)
          {
            USHORT depth = *pBuffer;

            // To convert to a byte, we're discarding the most-significant
            // rather than least-significant bits.
            // We're preserving detail, although the intensity will "wrap."
            // Values outside the reliable depth range are mapped to 0 (black).

            // Note: Using conditionals in this loop could degrade performance.
            // Consider using a lookup table instead when writing production code.
            BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? ((depth- nMinDepth) % 256) : 0);

            pRGBX->rgbRed = intensity;
            pRGBX->rgbGreen = 0;
            pRGBX->rgbBlue = 0;
            if (useDemux)
            {
              if (DemuxMethod == 1)
              {
                int corresPixelPos0 = j*pic.iPicWidth + i;
                int corresPixelPos1 = j*pic.iPicWidth + nWidth + i;
                int corresPixelPos2 = pic.iPicWidth*nHeight + j*pic.iPicWidth + i;
                int corresPixelPos3 = pic.iPicWidth*nHeight + j*pic.iPicWidth + nWidth + i;
                *(pDepth + corresPixelPos0) = 0;
                *(pDepth + corresPixelPos1) = 0;
                *(pDepth + corresPixelPos2) = 0;
                *(pDepth + corresPixelPos3) = 0;
                if ((depth >= nMinDepth) && (depth <= nMinDepth + 1020))
                {
                  if ((depth > nMinDepth) && (depth <= nMinDepth + 255))
                  {
                    *(pDepth + corresPixelPos0) = depth - nMinDepth;
                  }
                  else if ((depth > nMinDepth + 255) && (depth <= nMinDepth + 510))
                  {
                    *(pDepth + corresPixelPos1) = depth - nMinDepth - 255;
                  }
                  else if ((depth > nMinDepth + 510) && (depth <= nMinDepth + 765))
                  {
                    *(pDepth + corresPixelPos2) = depth - nMinDepth - 510;
                  }
                  else if ((depth > nMinDepth + 765) && (depth <= nMinDepth + 1020))
                  {
                    *(pDepth + corresPixelPos3) = depth - nMinDepth - 765;
                  }
                }
              }
              else if (DemuxMethod == 2)
              {
                *(pDepth + j*pic.iPicWidth + i) = 0;
                *(pDepth + j*pic.iPicWidth + i + pic.iPicWidth*pic.iPicHeight) = 0;
                if ((depth >= nMinDepth) && (depth <= nMaxDepth))
                {
                  *(pDepth + j*pic.iPicWidth + i) = intensity;
                  *(pDepth + j*pic.iPicWidth + i + pic.iPicWidth*pic.iPicHeight) = (depth- nMinDepth)/256+1;
                }
              }
              else if (DemuxMethod == 3)
              {
                *(pDepth + j*pic.iPicWidth + i) = 0;
                *(pDepthIndex + j*pic.iPicWidth + i)= 0;
                if ((depth >= nMinDepth) && (depth <= nMaxDepth))
                {
                  *(pDepth + j*pic.iPicWidth + i) = intensity;
                  *(pDepthIndex + j*pic.iPicWidth + i) = (depth - nMinDepth) / 256 + 1;
                }
              }
            }
            else
            {
              *(pDepth+j*pic.iPicWidth + i) = intensity;
            }
            ++pRGBX;
            ++pBuffer;
          }
        }
        if ((useDemux == true) && (DemuxMethod == 2))
        {
        }
        else
        {
          for (int i = 0; i < (pic.iPicWidth * pic.iPicHeight) / 2; i++)
          {
            *(pDepth + pic.iPicWidth * pic.iPicHeight + i) = 0;
          }
          for (int i = 0; i < (picIndex.iPicWidth * picIndex.iPicHeight) / 2; i++)
          {
            *(pDepthIndex + picIndex.iPicWidth * picIndex.iPicHeight + i) = 0;
          }
        }
        // Draw the data with Direct2D
        //m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
        if (m_bSaveScreenshot)
        {
            WCHAR szScreenshotPath[MAX_PATH];

            // Retrieve the path to My Photos
            GetScreenshotFileName(szScreenshotPath, _countof(szScreenshotPath));

            // Write out the bitmap to disk
            HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(m_pDepthRGBX), nWidth, nHeight, sizeof(RGBQUAD) * 8, szScreenshotPath);

            WCHAR szStatusMessage[64 + MAX_PATH];
            if (SUCCEEDED(hr))
            {
                // Set the status bar to show where the screenshot was saved
                StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Screenshot saved to %s", szScreenshotPath);
            }
            else
            {
                StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Failed to write screenshot to %s", szScreenshotPath);
            }

            SetStatusMessage(szStatusMessage, 5000, true);

            // toggle off so we don't save a screenshot again next frame
            m_bSaveScreenshot = false;
        }
    }
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// </summary>
void CDepthBasics::ProcessColor(INT64 nTime, UINT16*pBuffer, RGBQUAD* pBufferColor, int nWidth, int nHeight, int nWidthColor, int nHeightColor)
{
  // Make sure we've received valid data
  if (m_pCoordinateMapper &&
    pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight) &&
    pBufferColor && (nWidthColor == cColorWidth) && (nHeightColor == cColorHeight))
  {
    HRESULT hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(cDepthWidth * cDepthHeight, (UINT16*)pBuffer, nWidthColor * nHeightColor, m_pDepthCoordinates);
    uint8_t RGBFrame[3 * cDepthWidth * cDepthHeight] = { 0 };
    //std::vector<std::vector<uint8_t> >testTemp(cDepthWidth * cDepthHeight, std::vector<uint8_t>(3,0));
    if (SUCCEEDED(hr))
    {
      // loop over output pixels
      RGBQUAD* pSrc = new RGBQUAD();
      int validPoint = 0;
      for (int colorIndex = 0; colorIndex < (nWidthColor*nHeightColor); ++colorIndex)
      {
        // default setting source to copy from the background pixel

        DepthSpacePoint p = m_pDepthCoordinates[colorIndex];

        // Values that are negative infinity means it is an invalid color to depth mapping so we
        // skip processing for this pixel
        if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
        {
          int depthX = static_cast<int>(p.X + 0.5f);
          int depthY = static_cast<int>(p.Y + 0.5f);

         
          if ((depthX >= 0 && depthX < nWidth) && (depthY >= 0 && depthY < nHeight))
          {
            validPoint++;
            int fillIndex = CheckNeighbors(RGBFrame, depthY*nWidth + depthX, nWidth,nHeight);
            // set source for copy to the color pixel
            //pSrc = m_pColorRGBX + colorIndex;
            if (fillIndex >= 0)
            {
              RGBFrame[3 * fillIndex] = (pBufferColor + colorIndex)->rgbRed;
              RGBFrame[3 * fillIndex + 1] = (pBufferColor + colorIndex)->rgbGreen;
              RGBFrame[3 * fillIndex + 2] = (pBufferColor + colorIndex)->rgbBlue;
              m_pDepthRGBX[fillIndex] = *(pBufferColor + colorIndex);
              pSrc->rgbBlue = 0;
              pSrc->rgbGreen = 0;
              pSrc->rgbRed = 0;
              pBufferColor[colorIndex] = *pSrc;
            }
          }
        }
      }
      pSrc = NULL;
      if (useCompressForRGB)
      {
        Bitmap2Yuv420p_calc2(m_pColorYUV420.data(), RGBFrame, nWidth, nHeight);
      }
      else
      {
        memcpy(m_pColorYUV420.data(), RGBFrame, nWidth*nHeight*3);
      }
    }
  }
  if (pBufferColor && (nWidthColor == cColorWidth) && (nHeightColor == cColorHeight))
  {
    // Draw the data with Direct2D
    m_pDrawColor->Draw(reinterpret_cast<BYTE*>(pBufferColor), cColorWidth * cColorHeight * sizeof(RGBQUAD));
  }
  m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CDepthBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Get the name of the file where screenshot will be stored.
/// </summary>
/// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
/// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT CDepthBasics::GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize)
{
    WCHAR* pszKnownPath = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &pszKnownPath);

    if (SUCCEEDED(hr))
    {
        // Get the time
        WCHAR szTimeString[MAX_PATH];
        GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", szTimeString, _countof(szTimeString));

        // File name will be KinectScreenshotDepth-HH-MM-SS.bmp
        StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\KinectScreenshot-Depth-%s.bmp", pszKnownPath, szTimeString);
    }

    if (pszKnownPath)
    {
        CoTaskMemFree(pszKnownPath);
    }

    return hr;
}

/// <summary>
/// Save passed in image data to disk as a bitmap
/// </summary>
/// <param name="pBitmapBits">image data to save</param>
/// <param name="lWidth">width (in pixels) of input image data</param>
/// <param name="lHeight">height (in pixels) of input image data</param>
/// <param name="wBitsPerPixel">bits per pixel of image data</param>
/// <param name="lpszFilePath">full file path to output bitmap to</param>
/// <returns>indicates success or failure</returns>
HRESULT CDepthBasics::SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
    DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

    BITMAPINFOHEADER bmpInfoHeader = {0};

    bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
    bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
    bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
    bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
    bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
    bmpInfoHeader.biPlanes      = 1;                         // Default
    bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

    BITMAPFILEHEADER bfh = {0};

    bfh.bfType    = 0x4D42;                                           // 'M''B', indicates bitmap
    bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
    bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

    // Create the file on disk to write to
    HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    // Return if error opening file
    if (NULL == hFile) 
    {
        return E_ACCESSDENIED;
    }

    DWORD dwBytesWritten = 0;
    
    // Write the bitmap file header
    if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the bitmap info header
    if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the RGB Data
    if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }    

    // Close the file
    CloseHandle(hFile);
    return S_OK;
}
