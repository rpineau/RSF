//
//  nexdome.cpp
//  NexDome X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2016.


#include "RSF.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#include <string.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif
#ifdef SB_WIN_BUILD
#include <time.h>
#endif


CRSF::CRSF()
{

    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nCurPos = 0;
    m_nTargetPos = 0;
    m_bMoving = false;
    m_bHoming = false;
    m_bAbort = false;
    
    
#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\RSFLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RSFLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RSFLog.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRSF New Constructor Called\n", timestamp);
    fprintf(Logfile, "[%s] [CRSF::CRSF] version %3.2f build 2020_08_28_09_35.\n", timestamp, DRIVER_VERSION);
    fflush(Logfile);
#endif

}

CRSF::~CRSF()
{
#ifdef	PLUGIN_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif
}

int CRSF::Connect(const char *pszPort)
{
    int nErr = PLUGIN_OK;

    if(!m_pSerx)
        return ERR_COMMNOLINK;

#ifdef PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CRSF::Connect Called %s\n", timestamp, pszPort);
	fflush(Logfile);
#endif

    // 9600 8N1
    nErr = m_pSerx->open(pszPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1");
    if( nErr == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return nErr;

#ifdef PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CRSF::Connect connected to %s\n", timestamp, pszPort);
	fflush(Logfile);
#endif
	// lets start from clean buffers.
    m_pSerx->purgeTxRx();
    return nErr;
}

void CRSF::Disconnect()
{
    if(m_bIsConnected && m_pSerx) {
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }

	m_bIsConnected = false;
    m_nCurPos = 0;
    m_nTargetPos = 0;
    m_bMoving = false;
    m_bHoming = false;
}

#pragma mark getters and setters
int CRSF::getPosition(int &nPosition)
{
    int nErr = PLUGIN_OK;
    float fPos;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    nErr = RSFCommand(":Fp#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    fPos = atof(szResp+3);
    nPosition = int(fPos*1000);
    m_nCurPos = nPosition;

    return nErr;
}


int CRSF::getMinPosLimit()
{
    return -8000;
}

int CRSF::getMaxPosLimit()
{
    return 8000;
}
int CRSF::getTemperature(double &dTemperature)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    nErr = RSFCommand(":Ft1#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    dTemperature = atof(szResp+4);
    #ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRSF::getTemperature] dTemperature : %3.2f\n", timestamp, dTemperature);
        fflush(Logfile);
    #endif

    return nErr;
}

#pragma mark move commands
int CRSF::gotoPosition(int nPos)
{
    int nErr;
    char szCmd[SERIAL_BUFFER_SIZE];

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;


    if (nPos<-8000 || nPos>8000 )
        return ERR_LIMITSEXCEEDED;

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRSF::gotoPosition] goto position  : %+04d\n", timestamp, nPos);
    fflush(Logfile);
#endif

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Fm%+04d#", nPos);
    nErr = RSFCommand(szCmd, NULL, SERIAL_BUFFER_SIZE);
    if(nErr) {
        #ifdef PLUGIN_DEBUG
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRSF::gotoPosition] goto position  error : %d\n", timestamp, nErr);
            fflush(Logfile);
        #endif
        return nErr;
    }
    m_nTargetPos = nPos;
    m_bMoving = true;
    
    return nErr;
}

int CRSF::moveRelativeToPosision(int nSteps)
{
    int nErr;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRSF::moveRelativeToPosision] move relative position  : %+d\n", timestamp, nSteps);
    fflush(Logfile);
#endif

    m_nTargetPos = m_nCurPos + nSteps;
    nErr = gotoPosition(m_nTargetPos);
    return nErr;
}

int CRSF::goHome()
{
    int nErr;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRSF::goHome] Start homing\n", timestamp);
    fflush(Logfile);
#endif

    nErr = RSFCommand(":Fh#", NULL, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_bHoming = true;
    return nErr;
}

int CRSF::Abort()
{
    int nErr = PLUGIN_OK;
    
    #ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRSF::Abort] tring to stop all\n", timestamp);
        fflush(Logfile);
    #endif
    m_bHoming = false;
    m_bMoving = false;
    m_bAbort = true;
    
    m_nTargetPos = m_nCurPos;

    return nErr;
}

#pragma mark command complete functions

int CRSF::isGoToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    bool bMoving = false;
    int minBound, maxBound;
    
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;
    
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRSF::isGoToComplete]\n", timestamp);
    fflush(Logfile);
#endif

    if(m_bAbort) {
        bComplete = true;
        m_nTargetPos = m_nCurPos;
        m_bAbort = false;
        return nErr;
    }
    
    bComplete = false;
    nErr = isMotorMoving(bMoving);
    if(nErr) {
        #ifdef PLUGIN_DEBUG
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRSF::isGoToComplete] error when calling isMotorMoving()\n", timestamp);
            fflush(Logfile);
        #endif
        return ERR_CMDFAILED;
    }
    if(bMoving)
        return nErr;
    
    getPosition(m_nCurPos);
    minBound = m_nCurPos - 10;
    maxBound = m_nCurPos + 10;
    if(minBound <= m_nTargetPos && m_nTargetPos <= maxBound)
        bComplete = true;
    else
        bComplete = false;

    #ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRSF::isGoToComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
        fflush(Logfile);
    #endif

    return nErr;
}

int CRSF::isHomingComplete(bool &bHomingComplete)
{
    int nErr = PLUGIN_OK;
    bool bMoving;

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRSF::isHomingComplete]\n", timestamp);
    fflush(Logfile);
#endif

    if(m_bAbort) {
        bHomingComplete = false;
        m_nTargetPos = m_nCurPos;
        m_bAbort = false;
        return nErr;
    }

    nErr = isMotorMoving(bMoving);
    
    if(bMoving)
        bHomingComplete = false;
    else {
        bHomingComplete = !m_bHoming;
    }
    #ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRSF::isHomingComplete] bHomingComplete = %s\n", timestamp, bHomingComplete?"True":"False");
        fflush(Logfile);
    #endif

    return nErr;
    
}

#pragma mark motor status command

int CRSF::isMotorMoving(bool &bMoving)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
	
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRSF::isMotorMoving]\n", timestamp);
    fflush(Logfile);
#endif

    
    bMoving = m_bMoving;

    nErr = RSFCommand(":Fs#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strstr(szResp,"FS0")) {
        m_bMoving = false;
    }
    else if (strstr(szResp,"FS1")) {
        m_bMoving = true;
    }
    else {
        #ifdef PLUGIN_DEBUG
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRSF::isMotorMoving] error szResp = %s \n", timestamp, szResp);
            fflush(Logfile);
        #endif
        nErr = ERR_CMDFAILED;
    }
    bMoving = m_bMoving;
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRSF::isMotorMoving] bMoving = %s\n", timestamp, bMoving?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}





#pragma mark command and response functions

int CRSF::RSFCommand(const char *pszszCmd, char *pszResult, int nResultMaxLen)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;
	
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

#ifdef PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CRSF::RSFCommand Sending %s\n", timestamp, pszszCmd);
	fflush(Logfile);
#endif
    nErr = m_pSerx->writeFile((void *)pszszCmd, strlen(pszszCmd), ulBytesWrite);
    m_pSerx->flushTx();

    if(nErr){
        return nErr;
    }

    if(pszResult) {
        // read response
        if(m_bMoving || m_bHoming) { // check if we got an async response
            nErr = readAllResponses(szResp, SERIAL_BUFFER_SIZE);
        }
        else {
            nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
        }

#ifdef PLUGIN_DEBUG
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [CRSF::RSFCommand] response \"%s\"\n", timestamp, szResp);
		fflush(Logfile);
#endif
        // printf("Got response : %s\n",resp);
        strncpy(pszResult, szResp, nResultMaxLen);
#ifdef PLUGIN_DEBUG
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [CRSF::RSFCommand] response copied to pszResult : \"%s\"\n", timestamp, pszResult);
		fflush(Logfile);
#endif
    }
    return nErr;
}

int CRSF::readResponse(char *pszRespBuffer, int nBufferLen, int nTimeout)
{
    int nErr = PLUGIN_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;
	
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRSF::readResponse] ****************\n", timestamp);
            fflush(Logfile);
#endif

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, nTimeout);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRSF::readResponse] readFile Error : errno = %d \n", timestamp, nErr);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
			ltime = time(NULL);
			timestamp = asctime(localtime(&ltime));
			timestamp[strlen(timestamp) - 1] = 0;
			fprintf(Logfile, "[%s] [CRSF::readResponse] readFile Timeout.\n", timestamp);
			fflush(Logfile);
#endif
            nErr = ERR_NORESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 4
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRSF::readResponse] ulBytesRead = %lu\n", timestamp, ulBytesRead);
        fflush(Logfile);
#endif
    } while (*pszBufPtr++ != '#' && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead)
        *(pszBufPtr-1) = 0; //remove the #

    return nErr;
}

int CRSF::readAllResponses(char *respBuffer, unsigned int bufferLen)
{
    int nErr = PLUGIN_OK;
    int nbByteWaiting = 0;
    char szResp[SERIAL_BUFFER_SIZE];
    int nbTimeout = 0;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRSF::readAllResponses] ++++++++++++++++\n", timestamp);
    fflush(Logfile);
#endif

    memset(respBuffer, 0, bufferLen);
    do {
        m_pSerx->bytesWaitingRx(nbByteWaiting);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRSF::readAllResponses] nbByteWaiting = %d\n", timestamp, nbByteWaiting);
        fflush(Logfile);
#endif
        if(nbByteWaiting) {
            memset(szResp, 0, SERIAL_BUFFER_SIZE);
            nErr = readResponse(szResp, bufferLen);
    #ifdef PLUGIN_DEBUG
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRSF::readAllResponses] response \"%s\"\n", timestamp, szResp);
            fflush(Logfile);
    #endif
            if(strstr(szResp,":FM")) {// goto is done
                m_bMoving = false;
            }
            else if(strstr(szResp,":FH")) {// homing is done
                m_bHoming = false;
            }
            else {
                strncpy(respBuffer, szResp, bufferLen);
                return nErr;
            }
        } else { // we might need to wait a bit
            m_pSleeper->sleep(100);
            nbTimeout++;
            #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CRSF::readAllResponses] nbTimeout = %d\n", timestamp, nbTimeout);
                    fflush(Logfile);
            #endif

        }
    } while(nbTimeout<3);

    return nErr;
}


