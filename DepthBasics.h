//------------------------------------------------------------------------------
// <copyright file="DepthBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "ImageRenderer.h"
#include "DepthImageServer.cxx"

void Bitmap2Yuv420p_calc2(uint8_t *destination, uint8_t *rgb, size_t width, size_t height)
{
  size_t image_size = width * height;
  size_t upos = image_size;
  size_t vpos = upos + upos / 4;
  size_t i = 0;

  for (size_t line = 0; line < height; ++line)
  {
    if (!(line % 2))
    {
      for (size_t x = 0; x < width; x += 2)
      {
        uint8_t r = rgb[3 * i];
        uint8_t g = rgb[3 * i + 1];
        uint8_t b = rgb[3 * i + 2];

        destination[i] = (0.183 * r + 0.614 * g + 0.062* b ) + 16;
        destination[i] = (destination[i] < 16) ? 16 : destination[i];
        destination[i] = (destination[i] > 235) ? 235 : destination[i];
        destination[upos] = (-0.101 * r + (-0.339 * g) + 0.439 * b ) + 128;
        destination[vpos] = (0.439 * r + (-0.339 * g) + (-0.04 * b)) + 128;
        destination[upos] = (destination[upos] < 16) ? 16 : destination[upos];
        destination[upos] = (destination[upos] > 240) ? 240 : destination[upos];
        destination[vpos] = (destination[vpos] < 16) ? 16 : destination[vpos];
        destination[vpos] = (destination[vpos] > 240) ? 240 : destination[vpos];
        i++;
        upos++;
        vpos++;

        r = rgb[3 * i];
        g = rgb[3 * i + 1];
        b = rgb[3 * i + 2];

        destination[i] = (0.183 * r + 0.614 * g + 0.062* b) + 16;
        destination[i] = (destination[i] < 16) ? 16 : destination[i];
        destination[i] = (destination[i] > 235) ? 235 : destination[i];
        i++;
      }
    }
    else
    {
      for (size_t x = 0; x < width; x += 1)
      {
        uint8_t r = rgb[3 * i];
        uint8_t g = rgb[3 * i + 1];
        uint8_t b = rgb[3 * i + 2];

        destination[i] = (0.183 * r + 0.614 * g + 0.062* b) + 16;
        destination[i] = (destination[i] < 16) ? 16 : destination[i];
        destination[i] = (destination[i] > 235) ? 235 : destination[i];
        i++;
      }
    }
  }
};

class CDepthBasics
{
    static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;
    static const int        cColorWidth = 1920;
    static const int        cColorHeight = 1080;

public:
    /// <summary>
    /// Constructor
    /// </summary>
    CDepthBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CDepthBasics();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
    HWND                    m_hWnd;
    INT64                   m_nStartTime;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    INT64                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;
    bool                    m_bSaveScreenshot;

    bool _useDemux;
    // Current Kinect
    IKinectSensor*          m_pKinectSensor;

    // Coordinate Mapper
    ICoordinateMapper*      m_pCoordinateMapper;

    // Depth reader
    IDepthFrameReader*      m_pDepthFrameReader;

    // Color reader
    IColorFrameReader*      m_pColorFrameReader;

    IMultiSourceFrameReader* m_pMultiSourceReader;

    // Direct2D
    ImageRenderer*          m_pDrawDepth;
    ImageRenderer*          m_pDrawColor;
    ID2D1Factory*           m_pD2DFactory;
    RGBQUAD*                m_pDepthRGBX;
    RGBQUAD*                m_pColorRGBX;

    DepthSpacePoint *m_pDepthCoordinates;
    BufferedData m_pDepthYUV420;
    SFrameBSInfo info;
    SSourcePicture pic;

    BufferedData m_pColorYUV420;
    SFrameBSInfo infoColor;
    SSourcePicture picColor;

    igtl::MultiThreader::Pointer threaderServer;
    igtl::MutexLock::Pointer glockServer;
    DepthImageServer::ThreadDataServer td_Server;
    igtl::SimpleMutexLock *localMutex;

    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();

    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 InitializeDefaultSensor();

    /// <summary>
    /// Handle new depth data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="pBuffer">pointer to frame data</param>
    /// <param name="nWidth">width (in pixels) of input image data</param>
    /// <param name="nHeight">height (in pixels) of input image data</param>
    /// <param name="nMinDepth">minimum reliable depth</param>
    /// <param name="nMaxDepth">maximum reliable depth</param>
    /// </summary>
    void                    ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);

    /// <summary>
    /// Handle new depth data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="pBuffer">pointer to frame data</param>
    /// <param name="nWidth">width (in pixels) of input image data</param>
    /// <param name="nHeight">height (in pixels) of input image data</param>
    /// </summary>
    void                    ProcessColor(INT64 nTime, UINT16*pBuffer, RGBQUAD* pBufferColor, int nWidth, int nHeight, int nWidthColor, int nHeightColor);


    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    /// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
    /// <param name="bForce">force status update</param>
    bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

    /// <summary>
    /// Get the name of the file where screenshot will be stored.
    /// </summary>
    /// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
    /// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
    /// <returns>
    /// S_OK on success, otherwise failure code.
    /// </returns>
    HRESULT                 GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize);

    /// <summary>
    /// Save passed in image data to disk as a bitmap
    /// </summary>
    /// <param name="pBitmapBits">image data to save</param>
    /// <param name="lWidth">width (in pixels) of input image data</param>
    /// <param name="lHeight">height (in pixels) of input image data</param>
    /// <param name="wBitsPerPixel">bits per pixel of image data</param>
    /// <param name="lpszFilePath">full file path to output bitmap to</param>
    /// <returns>indicates success or failure</returns>
    HRESULT                 SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath);
};

