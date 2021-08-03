/*
FILENAME... SMCpegasusDriver.cpp
USAGE...    Motor driver support for the Micos SMC pegasus controller.

Note: This driver was tested with the Micos SMC pegasus controller and 
linear axis UPM-160, HPS-170 as well as rotary axis UPR-160 AIR. 
All axis were equipped with optical encoders. 

The limit-switches of linear axis are not working properly: once in 
the limit, one can't drive out of it, reason unknown. So the user 
provides a high hard limit for each axis (6th parameter of function 
'SMCpegasusCreateController'). This is set after the homerun, the 
controller won't accept anything above it (see Venus-2 command 
'setnlimit'). Additionally one can skip an axis that is not used or 
connected (7th paramter of function 'SMCpegasusCreateController').
*/

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstdlib>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "SMCpegasusDriver.h"

//#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new SMCpegasusController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCpegasusPortName  The name of the drvAsynSerialPort that was created previously to connect to the SMC pegasus controller 
  * \param[in] numAxes           The last axis to count to
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  * \param[in] limitList         The high-limit for each axis, separated by coma
  * \param[in] skipList          String representing the axes: e.g. 'nnnnyy'. If 'n', the axis will be skipped
  */
SMCpegasusController::SMCpegasusController(const char *portName, const char *SMCpegasusPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod, const char *limitList, const char *skipList)
  :  asynMotorController(portName, numAxes, NUM_SMCPEGASUS_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  int limitListArr[numAxes];
  asynStatus status;
  SMCpegasusAxis *pAxis;
  static const char *functionName = "SMCpegasusController::SMCpegasusController";

  // Create controller-specific parameters
  //createParam(SMCpegasusRegulatorModeString, asynParamInt32, &SMCpegasusRegulatorMode_);
  
  // convert the limitList to integer array
  std::string str = limitList;
  
  std::stringstream text_stream(limitList);
  std::string item;
  
  int i = 0;
  while (std::getline(text_stream, item, ',')) {
      limitListArr[i] = std::atoi(item.c_str());
	  i++;
}

  std::string skipping = skipList;
  
  /* Connect to SMC pegasus controller */
  status = pasynOctetSyncIO->connect(SMCpegasusPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to SMC pegasus controller\n",
      functionName);
  }
  for (axis=0; axis<numAxes; axis++) {
    if (skipping[axis] == 'n')
		//printf("skipping axis %i \n", axis);
        continue;
	//printf("create axis %i \n", axis);
    pAxis = new SMCpegasusAxis(this, axis, limitListArr[axis]);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new SMCpegasusController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCpegasusPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC pegasus controller 
  * \param[in] numAxes           The last axis to count to
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  * \param[in] limitList         The high hard limits for each axis, controller won't accept anything above it 
  * \param[in] skipList          String representing the axes: e.g. 'nnnnyy'. If 'n', the axis will be skipped
  */
extern "C" int SMCpegasusCreateController(const char *portName, const char *SMCpegasusPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod, const char *limitList, const char *skipList)
{
  SMCpegasusController *pSMCpegasusController
    = new SMCpegasusController(portName, SMCpegasusPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000., limitList, skipList);
  pSMCpegasusController = NULL;
  return(asynSuccess);
}


/** Specify a new resolution for an SMC pegasus axis.
  * Configuration command, called directly or from iocsh
  * \param[in] SMCpegasusPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC pegasus controller 
  * \param[in] axisNo            Index of the desired axis 
  * \param[in] newResolution     The new resolution of the specified axis
  */
extern "C" int SMCpegasusChangeResolution(const char *SMCpegasusPortName, int axisNo, double newResolution)
{
  SMCpegasusController *pC;
  static const char *functionName = "SMCpegasusChangeResolution";
  
  pC = (SMCpegasusController*) findAsynPortDriver(SMCpegasusPortName);
  if (!pC) {
    printf("SMCpegasusDriver.cpp:%s: Error port %s not found\n",
           functionName, SMCpegasusPortName);
    return asynError;
  }
  
  pC->lock();
  pC->changeResolution(axisNo, newResolution);
  pC->unlock();
  
  return(asynSuccess);
}

/** Change the resolution of an axis
  * \param[in] axisNo The index of the axis
  * \param[in] newResolution The new resolution
  */
asynStatus SMCpegasusController::changeResolution(int axisNo, double newResolution)
{
  SMCpegasusAxis* pAxis;
  asynStatus status;
  
  pAxis = this->getAxis(axisNo);
  
  status = pAxis->changeResolution(newResolution);
  
  return status;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void SMCpegasusController::report(FILE *fp, int level)
{
  fprintf(fp, "SMC pegasus motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an SMCpegasusAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
SMCpegasusAxis* SMCpegasusController::getAxis(asynUser *pasynUser)
{
  return static_cast<SMCpegasusAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SMCpegasusAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
SMCpegasusAxis* SMCpegasusController::getAxis(int axisNo)
{
  return static_cast<SMCpegasusAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the SMCpegasusAxis methods

/** Creates a new SMCpegasusAxis object.
  * \param[in] pC Pointer to the SMCpegasusController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
SMCpegasusAxis::SMCpegasusAxis(SMCpegasusController *pC, int axisNo, int highLimit)
  : asynMotorAxis(pC, axisNo),
    pC_(pC), highLimit_(highLimit)
{ 
  // identify the axis
  sprintf(pC_->outString_, "%i nidentify", (axisNo + 1));
  pC_->writeReadController();
  strcpy(identify_, pC_->inString_);
  
  // distinguish between linear and rotation axis by reading the polepairs
  // we want the limit switches off for rotation axis (see home and poll functions)
  sprintf(pC_->outString_, "%i getpolepairs", (axisNo_ + 1));
  pC_->writeReadController();
  polePairs_ = atoi( (char *) &pC_->inString_ );
    
  sprintf(pC_->outString_, "%i getpitch", (axisNo + 1));
  pC_->writeReadController();
  pitch_ = atof( (char *) &pC_->inString_ );

  sprintf(pC_->outString_, "%i getclperiod", (axisNo + 1));
  pC_->writeReadController();
  clPeriod_ = atof( (char *) &pC_->inString_ );
  
  // make shure that mres field is set to the same value !!
  axisRes_ = .0001;

  // pegasus supports closed-loop support, enable the StatusGainSupport_ bit
  // to switch closed-loop off/on with the CNEN field 
  setIntegerParam(pC_->motorStatusGainSupport_, 1);
  
  // this driver was tested with axes containing external encoders
  setIntegerParam(pC->motorStatusHasEncoder_, 1);
}
/** Change the axis resolution
  * \param[in] newResolution The new resolution
  */
asynStatus SMCpegasusAxis::changeResolution(double newResolution)
{
  axisRes_ = newResolution;
  
  return asynSuccess;
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void SMCpegasusAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
    fprintf(fp, "  identify %s\n", identify_);
    fprintf(fp, "    polePairs %d\n", polePairs_);
    fprintf(fp, "    pitch %f\n", pitch_);
    fprintf(fp, "    clPeriod %f\n", clPeriod_);
    fprintf(fp, "    axisRes %f\n", axisRes_);
    fprintf(fp, "    posTravelLimit %d\n", posTravelLimit_);
    fprintf(fp, "    negTravelLimit %d\n", negTravelLimit_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus SMCpegasusAxis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  // static const char *functionName = "SMCpegasusAxis::sendAccelAndVelocity";

  // Send the velocity
  sprintf(pC_->outString_, "%f %i snv", fabs(velocity * axisRes_), (axisNo_ + 1));
  status = pC_->writeController();

  // Send the acceleration
  // acceleration is in units/sec/sec
  sprintf(pC_->outString_, "%f %i sna", fabs(acceleration * axisRes_), (axisNo_ + 1));
  status = pC_->writeController();
  return status;
}


asynStatus SMCpegasusAxis::move(double position, int relative, double baseVelocity, double slewVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "SMCpegasusAxis::move";

  status = sendAccelAndVelocity(acceleration, slewVelocity);
  
  if (relative) {
    sprintf(pC_->outString_, "%f %i nr", (position * axisRes_), (axisNo_ + 1));
  } else {
    sprintf(pC_->outString_, "%f %i nm", (position * axisRes_), (axisNo_ + 1));
  }
  
  status = pC_->writeController();
  return status;
}

asynStatus SMCpegasusAxis::home(double baseVelocity, double slewVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "SMCpegasusAxis::home";

  status = sendAccelAndVelocity(acceleration, slewVelocity);
  
  // the UPR-1 60 F AIR rotation axis has 7 polepairs, the UPM-160 linear axis has 50
  // homing to the rm-switch blocks the controller-communication, so we always home to the cal-switch
  
  if (polePairs_ == 7) {
    sprintf(pC_->outString_, "1 0 %i setsw 1 1 %i setsw 1 0 %i sncal", (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1));
    rotHomeInProgress_ = true;
  } else {
    sprintf(pC_->outString_, "0 0 %i setsw 0 1 %i setsw 0 0 %i sncal 0.1 2 %i setncalvel", (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1));
    linHomeInProgress_ = true;
  }
  
  status = pC_->writeController();
  return status;
}

asynStatus SMCpegasusAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "SMCpegasusAxis::stop";

  sprintf(pC_->outString_, "%i nabort", (axisNo_ + 1));
  status = pC_->writeController();
  return status;
}

asynStatus SMCpegasusAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynSuccess;
  //static const char *functionName = "SMCpegasusAxis::setClosedLoop";

  // enable closed-loop control
  sprintf(pC_->outString_, "%i %i setcloop", closedLoop ? 1:0, (axisNo_ + 1));
  status = pC_->writeController();

  return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus SMCpegasusAxis::poll(bool *moving)
{ 
  int done=0;
  int axisStatus=-1;
  double position=0.0;
  asynStatus comStatus;

  //static const char *functionName = "SMCpegasusAxis::poll";
  
  // Read the current motor position
  sprintf(pC_->outString_, "%i np", (axisNo_ + 1));
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is a double
  position = atof( (char *) &pC_->inString_);
  setDoubleParam(pC_->motorPosition_, (position / axisRes_) );
  setDoubleParam(pC_->motorEncoderPosition_, (position / axisRes_) );

  // Read the status of this motor
  sprintf(pC_->outString_, "%i nst", (axisNo_ + 1));
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is an int
  axisStatus = atoi( (char *) &pC_->inString_);

  // Check the moving bit
  done = !(axisStatus & 0x1);
  setIntegerParam(pC_->motorStatusDone_, done);
  setIntegerParam(pC_->motorStatusMoving_, !done);
  *moving = done ? false:true;
  
  if (done) {
      
        // if it is a linear axis, check the hard-limits by position
        if (polePairs_ > 7) {
            if (position <= axisRes_) {
                setIntegerParam(pC_->motorStatusLowLimit_, 1);
            } else {
                setIntegerParam(pC_->motorStatusLowLimit_, 0);
            }
            if (position >= highLimit_) {
                setIntegerParam(pC_->motorStatusHighLimit_, 1);
            } else {
                setIntegerParam(pC_->motorStatusHighLimit_, 0);
            }
        }
        
        setIntegerParam(pC_->motorStatusAtHome_, 0);
      
        // Check if the axis was in homing-sequence
        if (rotHomeInProgress_) {
            rotHomeInProgress_ = false;
			// we need to wait a bit to let the axis come out of the limit switch
            epicsThreadSleep(5.);
            // deactivate the limit-switches for the rotation axis
            sprintf(pC_->outString_, "2 0 %i setsw 2 1 %i setsw", (axisNo_ + 1), (axisNo_ + 1));
            pC_->writeController();
            
            sprintf(pC_->outString_, "%i np", (axisNo_ + 1));
            pC_->writeReadController();
            // The response string is a double
            position = atof( (char *) &pC_->inString_);
            setDoubleParam(pC_->motorPosition_, (position / axisRes_) );
            setDoubleParam(pC_->motorEncoderPosition_, (position / axisRes_) );
            
            setIntegerParam(pC_->motorStatusAtHome_, 1);
        }
        
        if (linHomeInProgress_) {
            linHomeInProgress_ = false;
			// we need to wait a bit to let the axis come out of the limit switch
            epicsThreadSleep(5.);
            // the limit-switches for linear axis are not working properly: once in the limit, one can't drive out of it...
            // so we set the maximum travel-limit by giving the limitList string to the SMCpegasusCreateController function
            // the low limit shall always be 0
            sprintf(pC_->outString_, "0 %i %i setnlimit", highLimit_, (axisNo_ + 1));
            pC_->writeController();
            
            negTravelLimit_ = 0;
            posTravelLimit_ = highLimit_;
            
            sprintf(pC_->outString_, "%i np", (axisNo_ + 1));
            pC_->writeReadController();
            // The response string is a double
            position = atof( (char *) &pC_->inString_);
            setDoubleParam(pC_->motorPosition_, (position / axisRes_) );
            setDoubleParam(pC_->motorEncoderPosition_, (position / axisRes_) );
            
            setIntegerParam(pC_->motorStatusAtHome_, 1);
        }
        
        // Read status of switches 0=inactive 1=active
        // Note: calibration switch = low limit; range measure switch = high limit
        // Don't poll the limit switches while moving, or we'll block the controller's interpreter
        /*sprintf(pC_->outString_, "%i getswst", (axisNo_ + 1));
        pC_->writeReadController();
        // The response string is of the form "0 0"
        sscanf(pC_->inString_, "%i %i", &lowLimit, &highLimit);
        
        setIntegerParam(pC_->motorStatusLowLimit_, lowLimit);
        setIntegerParam(pC_->motorStatusHighLimit_, highLimit);
        
        if (lowLimit || highLimit) {
            limitActive_ = true;
        }*/
  }
  
  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg SMCpegasusCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SMCpegasusCreateControllerArg1 = {"SMC pegasus port name", iocshArgString};
static const iocshArg SMCpegasusCreateControllerArg2 = {"The last axis to count to", iocshArgInt};
static const iocshArg SMCpegasusCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SMCpegasusCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg SMCpegasusCreateControllerArg5 = {"List with high limits", iocshArgString};
static const iocshArg SMCpegasusCreateControllerArg6 = {"String indicating skipping of axis", iocshArgString};
static const iocshArg * const SMCpegasusCreateControllerArgs[] = {&SMCpegasusCreateControllerArg0,
                                                             &SMCpegasusCreateControllerArg1,
                                                             &SMCpegasusCreateControllerArg2,
                                                             &SMCpegasusCreateControllerArg3,
                                                             &SMCpegasusCreateControllerArg4,
                                                             &SMCpegasusCreateControllerArg5,
                                                             &SMCpegasusCreateControllerArg6};
static const iocshFuncDef SMCpegasusCreateControllerDef = {"SMCpegasusCreateController", 7, SMCpegasusCreateControllerArgs};
static void SMCpegasusCreateControllerCallFunc(const iocshArgBuf *args)
{
  SMCpegasusCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval, args[6].sval);
}

static const iocshArg SMCpegasusChangeResolutionArg0 = {"SMC pegasus port name", iocshArgString};
static const iocshArg SMCpegasusChangeResolutionArg1 = {"Axis number", iocshArgInt};
static const iocshArg SMCpegasusChangeResolutionArg2 = {"Axis resolution", iocshArgDouble};
static const iocshArg * const SMCpegasusChangeResolutionArgs[] = {&SMCpegasusChangeResolutionArg0,
                                                             &SMCpegasusChangeResolutionArg1,
                                                             &SMCpegasusChangeResolutionArg2};
static const iocshFuncDef SMCpegasusChangeResolutionDef = {"SMCpegasusChangeResolution", 3, SMCpegasusChangeResolutionArgs};
static void SMCpegasusChangeResolutionCallFunc(const iocshArgBuf *args)
{
  SMCpegasusChangeResolution(args[0].sval, args[1].ival, args[2].dval);
}

static void SMCpegasusRegister(void)
{
  iocshRegister(&SMCpegasusCreateControllerDef, SMCpegasusCreateControllerCallFunc);
  iocshRegister(&SMCpegasusChangeResolutionDef, SMCpegasusChangeResolutionCallFunc);
}

extern "C" {
epicsExportRegistrar(SMCpegasusRegister);
}
