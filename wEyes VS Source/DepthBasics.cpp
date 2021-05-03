//------------------------------------------------------------------------------
// <copyright file="DepthBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "DepthBasics.h"
#include "hm_lut.h"
#include <winsock2.h>
#include <WS2tcpip.h>
#include <cmath>
#pragma comment (lib, "Ws2_32.lib")

char host_name[100] = {};

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
    
    // take the command line argument and convert it to an ascii string
    wcstombs(host_name, lpCmdLine, 100);

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
    m_pDepthRGBX(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

    // create heap storage for depth pixel data in RGBX format
    m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
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

    if (m_pDepthRGBX)
    {
        delete [] m_pDepthRGBX;
        m_pDepthRGBX = NULL;
    }

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with depth frame reader
    SafeRelease(m_pDepthFrameReader);

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

    // Initialize a socket to communicate with the MOOS app
    WSADATA wsa_data;
    WSAStartup(MAKEWORD(1, 1), &wsa_data);
    SOCKET sockFD = socket(AF_INET, SOCK_DGRAM, 0);

    // Handle any outstanding messages from Windows before calibration begins
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

    // Run the calibration function to train the software on the area
    Calibrate();

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        // Create a UDP packet buffer to hold the results of the collision detection check
        char packet[64] = {};


        if (Update(packet))
        {
            struct hostent* targetHost;
            struct sockaddr_in targetAddr;

            targetHost = gethostbyname(host_name);

            if (targetHost)
            {
                memmove((char*)&targetAddr.sin_addr.s_addr, (char*)targetHost->h_addr, targetHost->h_length);
                targetAddr.sin_family = AF_INET;
                targetAddr.sin_port = htons(9900);

                sendto(sockFD, (const char*)packet, 64, 0, (struct sockaddr*) & targetAddr, sizeof(targetAddr));
            }
        }

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

void CDepthBasics::Calibrate()
{
  
    double *samples = new double[900*512];
    for (unsigned int k = 0; k < 900 * 512; ++k) {
        samples[k] = 0.0;
    }

    for (unsigned int k = 0; k < 900;)
    {


        if (!m_pDepthFrameReader)
        {
            return;
        }

        IDepthFrame* pDepthFrame = NULL;

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
            UINT16* pBuffer = NULL;

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
                nDepthMaxDistance = USHRT_MAX;

                // Note:  If you wish to filter by reliable depth distance, uncomment the following line.
                //// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
            }

            if (SUCCEEDED(hr))
            {
                hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
            }

            if (SUCCEEDED(hr))
            {
                // Make sure we've received valid data
                if (pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
                {
                    unsigned long long colIndex = 0;

                    // end pixel is start + width*height - 1
                    const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

                    while (pBuffer < pBufferEnd)
                    {
                        USHORT depth = *pBuffer;

                        // To convert to a byte, we're discarding the most-significant
                        // rather than least-significant bits.
                        // We're preserving detail, although the intensity will "wrap."
                        // Values outside the reliable depth range are mapped to 0 (black).

                        // Note: Using conditionals in this loop could degrade performance.
                        // Consider using a lookup table instead when writing production code.
                        USHORT intensity = ((depth >= nDepthMinReliableDistance) && (depth <= nDepthMaxDistance) ? depth : 0);

                        samples[(k * 512) + (colIndex % 512)] += (double)intensity / 424.0;
                        ++pBuffer;
                        ++colIndex;
                    }

                    ++k;

                }
            }

            SafeRelease(pFrameDescription);
        }

        SafeRelease(pDepthFrame);

    }
    // Calculate the mean of the 900 samples of column averages
    for (unsigned int k = 0; k < 900; ++k)
    {
        for (unsigned int j = 0; j < 512; ++j)
        {
            m_fBaseAvg[j] += samples[k*512 + j] / 900.0;
        }
    }

    // Calculate the standard deviation of the 900 samples
    for (unsigned int k = 0; k < 512; ++k)
    {
        double colVar = 0.0;

        for (unsigned int j = 0; j < 900; ++j)
        {
            double diff = samples[j*512 + k] - m_fBaseAvg[k];
            colVar += (diff * diff) / 900.0; 
        }
        m_fBaseStdDev[k] = sqrt(colVar);
    }
    delete[] samples;
}



/// <summary>
/// Main processing function
/// </summary>
bool CDepthBasics::Update(char * packet)
{

    bool retVal = false;

    if (!m_pDepthFrameReader)
    {
        return retVal;
    }

    IDepthFrame* pDepthFrame = NULL;

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
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
            //// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
        }

        if (SUCCEEDED(hr))
        {
            ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance, packet);
            retVal = true;
        }

        SafeRelease(pFrameDescription);
    }

    SafeRelease(pDepthFrame);

    return retVal;
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
            HRESULT hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cDepthWidth, cDepthHeight, cDepthWidth * sizeof(RGBQUAD)); 
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
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

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

        SafeRelease(pDepthFrameSource);
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
void CDepthBasics::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth, char* packet)
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

        // Make sure we've received valid data
        if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
        {
            RGBQUAD* pRGBX = m_pDepthRGBX;

            double colAvgs[512] = {};
            unsigned long long colIndex = 0;

            // end pixel is start + width*height - 1
            const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

            while (pBuffer < pBufferEnd)
            {
                USHORT depth = *pBuffer;

                // To convert to a byte, we're discarding the most-significant
                // rather than least-significant bits.
                // We're preserving detail, although the intensity will "wrap."
                // Values outside the reliable depth range are mapped to 0 (black).

                // Note: Using conditionals in this loop could degrade performance.
                // Consider using a lookup table instead when writing production code.
                USHORT intensity = ((depth >= nMinDepth) && (depth <= nMaxDepth) ? depth : 0);

                int RGBval = hm_table[intensity];

                pRGBX->rgbRed   = RGBval & 255;
                pRGBX->rgbGreen = (RGBval >> 8) & 255;
                pRGBX->rgbBlue  = (RGBval >> 16) & 255;

                colAvgs[colIndex % 512] += (double)intensity / 424.0;

                ++pRGBX;
                ++pBuffer;
                ++colIndex;
            }

        
            for (unsigned int i = 0; i < 512; ++i)
            {
                for (unsigned int j = 408; j < 416; ++j)
                {
                    RGBQUAD* newRGBX = m_pDepthRGBX + j * 512 + i;
                    newRGBX->rgbRed = (unsigned char)(colAvgs[i] / 4500.0 * 255.0);
                    newRGBX->rgbGreen = 0;
                    newRGBX->rgbBlue = 0;
                }
            }


            bool anomalies[512] = {};

            double baselineThreshPos[3] = {};
            double baselineThreshNeg[3] = {};

            baselineThreshPos[0] = m_fBaseAvg[0] + 5.0 * m_fBaseStdDev[0];
            baselineThreshPos[1] = m_fBaseAvg[1] + 5.0 * m_fBaseStdDev[1];
            baselineThreshNeg[0] = m_fBaseAvg[0] - 5.0 * m_fBaseStdDev[0];
            baselineThreshNeg[1] = m_fBaseAvg[1] - 5.0 * m_fBaseStdDev[1];


            anomalies[0] = ((colAvgs[0] > baselineThreshPos[0]) && (colAvgs[1] > baselineThreshPos[1])) ||
                           ((colAvgs[0] > baselineThreshPos[0]) && (colAvgs[1] < baselineThreshNeg[1])) ||
                           ((colAvgs[0] < baselineThreshNeg[0]) && (colAvgs[1] > baselineThreshPos[1])) ||
                           ((colAvgs[0] < baselineThreshNeg[0]) && (colAvgs[1] < baselineThreshNeg[1]));


            for (unsigned int i = 1; i < 511; ++i)
            {
                baselineThreshPos[(i + 1) % 3] = m_fBaseAvg[i + 1] + 5.0 * m_fBaseStdDev[i + 1];
                baselineThreshNeg[(i + 1) % 3] = m_fBaseAvg[i + 1] - 5.0 * m_fBaseStdDev[i + 1];

                anomalies[i] =  ((colAvgs[i] > baselineThreshPos[i % 3]) && (colAvgs[i - 1] > baselineThreshPos[(i - 1) % 3])) ||
                                ((colAvgs[i] > baselineThreshPos[i % 3]) && (colAvgs[i - 1] < baselineThreshNeg[(i - 1) % 3])) ||
                                ((colAvgs[i] < baselineThreshNeg[i % 3]) && (colAvgs[i - 1] > baselineThreshPos[(i - 1) % 3])) ||
                                ((colAvgs[i] < baselineThreshNeg[i % 3]) && (colAvgs[i - 1] < baselineThreshNeg[(i - 1) % 3])) ||
                                ((colAvgs[i] > baselineThreshPos[i % 3]) && (colAvgs[i + 1] > baselineThreshPos[(i + 1) % 3])) ||
                                ((colAvgs[i] > baselineThreshPos[i % 3]) && (colAvgs[i + 1] < baselineThreshNeg[(i + 1) % 3])) ||
                                ((colAvgs[i] < baselineThreshNeg[i % 3]) && (colAvgs[i + 1] > baselineThreshPos[(i + 1) % 3])) ||
                                ((colAvgs[i] < baselineThreshNeg[i % 3]) && (colAvgs[i + 1] < baselineThreshNeg[(i + 1) % 3]));

            }

            baselineThreshPos[0] = m_fBaseAvg[510] + 5.0 * m_fBaseStdDev[510];
            baselineThreshPos[1] = m_fBaseAvg[511] + 5.0 * m_fBaseStdDev[511];
            baselineThreshNeg[0] = m_fBaseAvg[510] - 5.0 * m_fBaseStdDev[510];
            baselineThreshNeg[1] = m_fBaseAvg[511] - 5.0 * m_fBaseStdDev[511];

            anomalies[511] =    ((colAvgs[510] > baselineThreshPos[0]) && (colAvgs[511] > baselineThreshPos[1])) ||
                                ((colAvgs[510] > baselineThreshPos[0]) && (colAvgs[511] < baselineThreshNeg[1])) ||
                                ((colAvgs[510] < baselineThreshNeg[0]) && (colAvgs[511] > baselineThreshPos[1])) ||
                                ((colAvgs[510] < baselineThreshNeg[0]) && (colAvgs[511] < baselineThreshNeg[1]));

            for (unsigned int i = 0; i < 512; ++i)
            {
                for (unsigned int j = 416; j < 424; ++j)
                {
                    RGBQUAD* newRGBX = m_pDepthRGBX + j * 512 + i;
                    newRGBX->rgbRed = (unsigned int)anomalies[i] * 255;
                    newRGBX->rgbGreen = 0;
                    newRGBX->rgbBlue = 0;
                }

                packet[i/8] = ((unsigned int)anomalies[i] << (7 - (i % 8))) ^ packet[i/8];

            }

            WCHAR szStatusMessage[64];
            StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

            if (SetStatusMessage(szStatusMessage, 1000, false))
            {
                m_nLastCounter = qpcNow.QuadPart;
                m_nFramesSinceUpdate = 0;
            }
        }
 
        // Draw the data with Direct2D
        m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));

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
