//
//  RSF.h
//  NexDome
//
//  Created by Rodolphe Pineau on 2017/05/30.
//  NexDome X2 plugin

#ifndef __RSF__
#define __RSF__
#include <math.h>
#include <string.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include <exception>
#include <typeinfo>
#include <stdexcept>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"

#define PLUGIN_DEBUG 2

#define DRIVER_VERSION      1.0

#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
#define RSF_LOGFILENAME "C:\\RSFLog.txt"
#elif defined(SB_LINUX_BUILD)
#define RSF_LOGFILENAME "/tmp/RSFLog.txt"
#elif defined(SB_MAC_BUILD)
#define RSF_LOGFILENAME "/tmp/RSFLog.txt"
#endif
#endif


#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 1000
#define LOG_BUFFER_SIZE 256

enum RSF_Errors    {PLUGIN_OK = 0, NOT_CONNECTED, ND_CANT_CONNECT, RSF_BAD_CMD_RESPONSE, COMMAND_FAILED};
enum MotorDir       {NORMAL = 0 , REVERSE};
enum MotorStatus    {IDLE = 0, MOVING};


class CRSF
{
public:
    CRSF();
    ~CRSF();

    int         Connect(const char *pszPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return m_bIsConnected; };

    void        SetSerxPointer(SerXInterface *p) { m_pSerx = p; };
    void        setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; };
    void        setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper; };

    // move commands
    int         gotoPosition(int nPos);
    int         moveRelativeToPosision(int nSteps);
    int         goHome(void);
    
    // command complete functions
    int         isGoToComplete(bool &bComplete);
    int         isMotorMoving(bool &bMoving);
    void        isHomingComplete(bool &bHoming);
    
    // getter and setter
    void        setDebugLog(bool bEnable) {m_bDebugLog = bEnable; };

    int         getTemperature(double &dTemperature);
    int         getPosition(int &nPosition);
    int         getMinPosLimit(void);
    int         getMaxPosLimit(void);

protected:

    int             RSFCommand(const char *pszCmd, char *pszResult, int nResultMaxLen);
    int             readResponse(char *pszRespBuffer, int nBufferLen, int nTimeout = MAX_TIMEOUT);
    int             readAllResponses(char *respBuffer, unsigned int bufferLen);
    
    SerXInterface   *m_pSerx;
    LoggerInterface *m_pLogger;
    SleeperInterface    *m_pSleeper;

    bool            m_bDebugLog;
    bool            m_bIsConnected;
    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    char            m_szLogBuffer[LOG_BUFFER_SIZE];

    int             m_nCurPos;
    int             m_nTargetPos;
    bool            m_bMoving;
    bool            m_bHoming;

#ifdef PLUGIN_DEBUG
	// timestamp for logs
	char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif

};

#endif //__RSF__
