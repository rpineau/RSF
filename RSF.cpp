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
    m_pLogger = NULL;


    m_bDebugLog = false;
    m_bIsConnected = false;

    m_nCurPos = 0;
    m_nTargetPos = 0;
    m_bMoving = false;
    m_bHoming = false;

#ifdef	PLUGIN_DEBUG
	Logfile = fopen(RSF_LOGFILENAME, "w");
	ltime = time(NULL);
	char *timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CRSF Constructor Called.\n", timestamp);
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
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    nErr = RSFCommand(":Fp#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    nPosition = atoi(szResp+3);
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
    char szTmpBuf[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    nErr = RSFCommand(":Ft1#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    dTemperature = atof(szTmpBuf+4);

    return nErr;
}

#pragma mark move commands
int CRSF::gotoPosition(int nPos)
{
    int nErr;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szTmpBuf[SERIAL_BUFFER_SIZE];

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;


    if (nPos<-8000 || nPos>8000 )
        return ERR_LIMITSEXCEEDED;

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRSF::gotoPosition] goto position  : %d\n", timestamp, nPos);
    fflush(Logfile);
#endif

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Fm%d#", nPos);
    nErr = RSFCommand(szCmd, NULL, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_nTargetPos = atoi(szTmpBuf);
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
    fprintf(Logfile, "[%s] CRSF::gotoPosition goto relative position  : %d\n", timestamp, nSteps);
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

#pragma mark command complete functions

int CRSF::isGoToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    bool bMoving = false;
    
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    bComplete = false;
    nErr = isMotorMoving(bMoving);
    if(nErr)
        return ERR_CMDFAILED;
    
    if(bMoving)
        return nErr;
    
    getPosition(m_nCurPos);
    if(m_nCurPos == m_nTargetPos)
        bComplete = true;
    else
        bComplete = false;
    return nErr;
}

void CRSF::isHomingComplete(bool &bHoming)
{
    bHoming = m_bHoming;
}

#pragma mark motor status command

int CRSF::isMotorMoving(bool &bMoving)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
	
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    if(m_bMoving) {
        nErr = RSFCommand(":Fs#", szResp, SERIAL_BUFFER_SIZE);
        if(nErr)
            return nErr;
        if(strstr(szResp,"FS0")) {
            m_bMoving = false;
        }
        else if (strstr(szResp,"FS1")) {
            m_bMoving = true;
        }
        else
            nErr = ERR_CMDFAILED;
    }
    bMoving = m_bMoving;
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

    if (m_bDebugLog && m_pLogger) {
        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CRSF::RSFCommand] Sending %s\n",pszszCmd);
        m_pLogger->out(m_szLogBuffer);
    }
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
        if (m_bDebugLog && m_pLogger) {
            snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CRSF::RSFCommand] writeFile Error.\n");
            m_pLogger->out(m_szLogBuffer);
        }
        return nErr;
    }

    if(pszResult) {
        // read response
        if (m_bDebugLog && m_pLogger) {
            snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CRSF::RSFCommand] Getting response.\n");
            m_pLogger->out(m_szLogBuffer);
        }
        if(m_bHoming || m_bHoming) { // check if we got an async response
            nErr = readAllResponses(szResp, SERIAL_BUFFER_SIZE);
        }
        else {
            nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
        }
        if(nErr){
            if (m_bDebugLog && m_pLogger) {
                snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CRSF::RSFCommand] readResponse Error.\n");
                m_pLogger->out(m_szLogBuffer);
            }
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

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, nTimeout);
        if(nErr) {
            if (m_bDebugLog && m_pLogger) {
                snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CRSF::readResponse] readFile Error.\n");
                m_pLogger->out(m_szLogBuffer);
            }
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
            if (m_bDebugLog && m_pLogger) {
                snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CRSF::readResponse] readFile Timeout.\n");
                m_pLogger->out(m_szLogBuffer);
            }
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
        if (m_bDebugLog && m_pLogger) {
            snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CRSF::readResponse] ulBytesRead = %lu\n",ulBytesRead);
            m_pLogger->out(m_szLogBuffer);
        }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
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

    memset(respBuffer, 0, bufferLen);
    do {
        m_pSerx->bytesWaitingRx(nbByteWaiting);
        if(nbByteWaiting) {
            memset(szResp, 0, SERIAL_BUFFER_SIZE);
            nErr = readResponse(szResp, bufferLen);
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
        }
    } while(nbByteWaiting);

    return nErr;
}


