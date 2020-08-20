#include <stdio.h>
#include <string.h>
#include "x2focuser.h"

#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/mutexinterface.h"
#include "../../licensedinterfaces/basicstringinterface.h"
#include "../../licensedinterfaces/tickcountinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serialportparams2interface.h"

X2Focuser::X2Focuser(const char* pszDisplayName, 
												const int& nInstanceIndex,
												SerXInterface						* pSerXIn, 
												TheSkyXFacadeForDriversInterface	* pTheSkyXIn, 
												SleeperInterface					* pSleeperIn,
												BasicIniUtilInterface				* pIniUtilIn,
												LoggerInterface						* pLoggerIn,
												MutexInterface						* pIOMutexIn,
												TickCountInterface					* pTickCountIn)

{
	m_pSerX							= pSerXIn;		
	m_pTheSkyXForMounts				= pTheSkyXIn;
	m_pSleeper						= pSleeperIn;
	m_pIniUtil						= pIniUtilIn;
	m_pLogger						= pLoggerIn;	
	m_pIOMutex						= pIOMutexIn;
	m_pTickCount					= pTickCountIn;
	
	m_bLinked = false;
	m_nPosition = 0;
    m_fLastTemp = -273.15f; // aboslute zero :)

    // Read in settings
    if (m_pIniUtil) {
    }
	m_RSFController.SetSerxPointer(m_pSerX);
	m_RSFController.setLogger(m_pLogger);
    m_RSFController.setSleeper(m_pSleeper);
}

X2Focuser::~X2Focuser()
{
    //Delete objects used through composition
	if (GetSerX())
		delete GetSerX();
	if (GetTheSkyXFacadeForDrivers())
		delete GetTheSkyXFacadeForDrivers();
	if (GetSleeper())
		delete GetSleeper();
	if (GetSimpleIniUtil())
		delete GetSimpleIniUtil();
	if (GetLogger())
		delete GetLogger();
	if (GetMutex())
		delete GetMutex();

}

#pragma mark - DriverRootInterface

int	X2Focuser::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LinkInterface_Name))
        *ppVal = (LinkInterface*)this;

    else if (!strcmp(pszName, FocuserGotoInterface2_Name))
        *ppVal = (FocuserGotoInterface2*)this;

    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);

    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);

    else if (!strcmp(pszName, FocuserTemperatureInterface_Name))
        *ppVal = dynamic_cast<FocuserTemperatureInterface*>(this);

    else if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();

    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);

    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);

    return SB_OK;
}

#pragma mark - DriverInfoInterface
void X2Focuser::driverInfoDetailedInfo(BasicStringInterface& str) const
{
        str = "Focuser X2 plugin by Rodolphe Pineau";
}

double X2Focuser::driverInfoVersion(void) const							
{
	return DRIVER_VERSION;
}

void X2Focuser::deviceInfoNameShort(BasicStringInterface& str) const
{
    str="AAF2";
}

void X2Focuser::deviceInfoNameLong(BasicStringInterface& str) const				
{
    deviceInfoNameShort(str);
}

void X2Focuser::deviceInfoDetailedDescription(BasicStringInterface& str) const		
{
	str = "AAF2 Controller";
}

void X2Focuser::deviceInfoFirmwareVersion(BasicStringInterface& str)				
{
    str="";
}

void X2Focuser::deviceInfoModel(BasicStringInterface& str)							
{
    str="AAF2";
}

#pragma mark - LinkInterface
int	X2Focuser::establishLink(void)
{
    char szPort[DRIVER_MAX_STRING];
    int nErr;

    X2MutexLocker ml(GetMutex());
    // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);
    nErr = m_RSFController.Connect(szPort);
    if(nErr)
        m_bLinked = false;
    else
        m_bLinked = true;

    return nErr;
}

int	X2Focuser::terminateLink(void)
{
    if(!m_bLinked)
        return SB_OK;

    X2MutexLocker ml(GetMutex());
    m_RSFController.Disconnect();
    m_bLinked = false;

	return SB_OK;
}

bool X2Focuser::isLinked(void) const
{
	return m_bLinked;
}

#pragma mark - ModalSettingsDialogInterface
int	X2Focuser::initModalSettingsDialog(void)
{
    return SB_OK;
}

int	X2Focuser::execModalSettingsDialog(void)
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;


    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("RSF.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());
	// set controls values
    if(m_bLinked) {
    }
    else {
    }

    //Display the user interface
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK) {
        nErr = SB_OK;
    }
    return nErr;
}

void X2Focuser::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    bool bHomingComplete = false;
    char szErrorMessage[LOG_BUFFER_SIZE];
    
    if (!strcmp(pszEvent, "on_timer")) {
        // is homing done ?
        if(m_bHoming) {
            bHomingComplete = false;
            nErr = m_RSFController.isHomingComplete(bHomingComplete);
            if(nErr) {
                uiex->setEnabled("pushButton",true);
                uiex->setEnabled("pushButtonOK",true);
                snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error homing focuser : Error %d", nErr);
                uiex->messageBox("RSF Homing", szErrorMessage);
                m_bHoming = false;
                return;;
            }
            
            if(!bHomingComplete) {
                return;
            }
            
            // enable "ok" and "calibrate"
            uiex->setEnabled("pushButton",true);
            uiex->setEnabled("pushButtonOK",true);
            m_bHoming = false;
        }
    }
    else if (!strcmp(pszEvent, "on_pushButton_clicked")) {
        // start homing
        if(m_bLinked) {
            // disable "ok" and "calibrate"
            uiex->setEnabled("pushButton",false);
            uiex->setEnabled("pushButtonOK",false);
            m_RSFController.goHome();
            m_bHoming = true;
        }
    }
}

#pragma mark - FocuserGotoInterface2
int	X2Focuser::focPosition(int& nPosition)
{
    int nErr;

    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());

    nErr = m_RSFController.getPosition(nPosition);
    m_nPosition = nPosition;
    return nErr;
}

int	X2Focuser::focMinimumLimit(int& nMinLimit) 		
{
    nMinLimit = m_RSFController.getMinPosLimit();
    return SB_OK;
}

int	X2Focuser::focMaximumLimit(int& nPosLimit)			
{

	X2MutexLocker ml(GetMutex());
    nPosLimit = m_RSFController.getMaxPosLimit();
	return SB_OK;
}

int	X2Focuser::focAbort()								
{

    if(!m_bLinked)
        return NOT_CONNECTED;

    return SB_OK;
}

int	X2Focuser::startFocGoto(const int& nRelativeOffset)	
{
    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    m_RSFController.moveRelativeToPosision(nRelativeOffset);
    return SB_OK;
}

int	X2Focuser::isCompleteFocGoto(bool& bComplete) const
{
    int nErr;

    if(!m_bLinked)
        return NOT_CONNECTED;

    X2Focuser* pMe = (X2Focuser*)this;
    X2MutexLocker ml(pMe->GetMutex());
	nErr = pMe->m_RSFController.isGoToComplete(bComplete);

    return nErr;
}

int	X2Focuser::endFocGoto(void)
{
    int nErr;
    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    nErr = m_RSFController.getPosition(m_nPosition);
    return nErr;
}

int X2Focuser::amountCountFocGoto(void) const					
{ 
	return 4;
}

int	X2Focuser::amountNameFromIndexFocGoto(const int& nZeroBasedIndex, BasicStringInterface& strDisplayName, int& nAmount)
{
	switch (nZeroBasedIndex)
	{
		default:
		case 0: strDisplayName="1 steps"; nAmount=10;break;
		case 1: strDisplayName="10 steps"; nAmount=100;break;
		case 2: strDisplayName="100 steps"; nAmount=1000;break;
        case 3: strDisplayName="1000 steps"; nAmount=1000;break;
	}
	return SB_OK;
}

int	X2Focuser::amountIndexFocGoto(void)
{
	return 0;
}

#pragma mark - FocuserTemperatureInterface
int X2Focuser::focTemperature(double &dTemperature)
{
    int nErr = SB_OK;

    if(!m_bLinked) {
        dTemperature = -100.0;
        return NOT_CONNECTED;
    }
    X2MutexLocker ml(GetMutex());

    // Taken from Richard's Robofocus plugin code.
    // this prevent us from asking the temperature too often
    static CStopWatch timer;

    if(timer.GetElapsedSeconds() > 30.0f || m_fLastTemp < -99.0f) {
        nErr = m_RSFController.getTemperature(m_fLastTemp);
        timer.Reset();
    }

    dTemperature = m_fLastTemp;

    return nErr;
}

#pragma mark - SerialPortParams2Interface

void X2Focuser::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Focuser::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort);

}


void X2Focuser::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize, DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
    
}




