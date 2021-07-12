/*
FILENAME... SMCpegasusDriver.cpp
USAGE...    Motor driver support for the Micos SMC pegasus controller.

Note: This driver was tested with the Micos SMC pegasus ...

*/


#include <stdio.h>
#include <string.h>
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
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SMCpegasusController::SMCpegasusController(const char *portName, const char *SMCpegasusPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_SMCPEGASUS_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  SMCpegasusAxis *pAxis;
  static const char *functionName = "SMCpegasusController::SMCpegasusController";

  // Create controller-specific parameters
  //createParam(SMCpegasusRegulatorModeString, asynParamInt32, &SMCpegasusRegulatorMode_);

  /* Connect to SMC pegasus controller */
  status = pasynOctetSyncIO->connect(SMCpegasusPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to SMC pegasus controller\n",
      functionName);
  }
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new SMCpegasusAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new SMCpegasusController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCpegasusPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC pegasus controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int SMCpegasusCreateController(const char *portName, const char *SMCpegasusPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  SMCpegasusController *pSMCpegasusController
    = new SMCpegasusController(portName, SMCpegasusPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
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
SMCpegasusAxis::SMCpegasusAxis(SMCpegasusController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{ 
  sprintf(pC_->outString_, "%i getpitch", (axisNo + 1));
  pC_->writeReadController();
  pitch_ = atof( (char *) &pC_->inString_ );
  
  sprintf(pC_->outString_, "%i getpolepairs", (axisNo + 1));
  pC_->writeReadController();
  polePairs_ = atoi( (char *) &pC_->inString_ );

  sprintf(pC_->outString_, "%i getclperiod", (axisNo + 1));
  pC_->writeReadController();
  clPeriod_ = atof( (char *) &pC_->inString_ );
  // Linear or torque motor or other motor forms
  axisRes_ = .0001;

  /*switch (motorForm_)
  {
    case 0:
      // Stepper motor
      axisRes_ = pitch_ / ( 4.0 * polePairs_);
      break;

    case 1:
      // Linear or torque motor
      axisRes_ = clPeriod_;
      break;

    default:
      // For now assume clPeriod_ works for other motor forms
      axisRes_ = clPeriod_;
      break;
  }*/

  /* Enable gain support so that the CNEN field can be used to send
     the init command to clear a motor fault for stepper motors, even
     though they lack closed-loop support. */
  //setIntegerParam(pC_->motorStatusGainSupport_, 1);
  
  //we have external encoders
  setIntegerParam(pC->motorStatusHasEncoder_, 1);

  // Determine the travel limits (will change after homing)
  sprintf(pC_->outString_, "%i getnlimit", (axisNo + 1));
  pC_->writeReadController();
  sscanf(pC_->inString_, "%lf %lf", &negTravelLimit_, &posTravelLimit_);

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
    //fprintf(fp, "    motorForm %d\n", motorForm_);
    fprintf(fp, "    pitch %f\n", pitch_);
    fprintf(fp, "    polePairs %d\n", polePairs_);
    fprintf(fp, "    clPeriod %f\n", clPeriod_);
    fprintf(fp, "    axisRes %f\n", axisRes_);
    fprintf(fp, "    lowLimitConfig %d\n", lowLimitConfig_);
    fprintf(fp, "    highLimitConfig %d\n", highLimitConfig_);
    fprintf(fp, "    posTravelLimit %f\n", posTravelLimit_);
    fprintf(fp, "    negTravelLimit %f\n", negTravelLimit_);
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
  //sprintf(pC_->outString_, "%f %i snv", 5., (axisNo_ + 1));
  status = pC_->writeController();

  // Send the acceleration
  // acceleration is in units/sec/sec
  sprintf(pC_->outString_, "%f %i sna", fabs(acceleration * axisRes_), (axisNo_ + 1));
  //sprintf(pC_->outString_, "%f %i sna", 0.5, (axisNo_ + 1));
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
    //sprintf(pC_->outString_, "%f %i nr", (position), (axisNo_ + 1));
  } else {
    sprintf(pC_->outString_, "%f %i nm", (position * axisRes_), (axisNo_ + 1));
    //sprintf(pC_->outString_, "%f %i nm", (position), (axisNo_ + 1));
  }
  status = pC_->writeController();
  return status;
}

asynStatus SMCpegasusAxis::home(double baseVelocity, double slewVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "SMCpegasusAxis::home";

  status = sendAccelAndVelocity(acceleration, slewVelocity);
  
  // distinguish between linear and rotation axis with polepairs
  // we want the limit switches off for rotation axis
  sprintf(pC_->outString_, "%i getpolepairs", (axisNo_ + 1));
  pC_->writeReadController();
  polePairs_ = atoi( (char *) &pC_->inString_ );
  
  // the UPR-1 60 F AIR rotation axis has 7 polepairs, the UPM-160 linear axis has 50
  // so far I don't know if positiv homing is possible with Pegasus, so we drive always negative
  if (forwards) {
	if (polePairs_ == 7) {
		sprintf(pC_->outString_, "1 0 %i setsw 1 1 %i setsw 1 0 %i sncal", (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1));
		homingInProgress_ = true;
	} else {
		sprintf(pC_->outString_, "0 0 %i setsw 0 1 %i setsw 0 0 %i sncal 0.1 2 %i setncalvel", (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1));
	}
  } else {
	if (polePairs_ == 7) {
		sprintf(pC_->outString_, "1 0 %i setsw 1 1 %i setsw 1 0 %i sncal", (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1));
		homingInProgress_ = true;
	} else {
		sprintf(pC_->outString_, "0 0 %i setsw 0 1 %i setsw 0 0 %i sncal 0.1 2 %i setncalvel", (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1));
	}
  }
  status = pC_->writeController();
  return status;
}

asynStatus SMCpegasusAxis::moveVelocity(double baseVelocity, double slewVelocity, double acceleration)
{
  asynStatus status;
  static const char *functionName = "SMCpegasusAxis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: baseVelocity=%f, slewVelocity=%f, acceleration=%f\n",
    functionName, baseVelocity, slewVelocity, acceleration);
  
  /* SMC pegasus does not have jog command. Move to a limit*/
  if (slewVelocity > 0.) {
    status = sendAccelAndVelocity(acceleration, slewVelocity);
    sprintf(pC_->outString_, "%f %i nm", posTravelLimit_, (axisNo_ + 1));
  } else {
    status = sendAccelAndVelocity(acceleration, (slewVelocity * -1.0));
    sprintf(pC_->outString_, "%f %i nm", negTravelLimit_, (axisNo_ + 1));
  }
  status = pC_->writeController();
  return status;
}

asynStatus SMCpegasusAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "SMCpegasusAxis::stop";

  // Set stop deceleration (will be overridden by accel if accel is higher)
  //sprintf(pC_->outString_, "%f %i ssd", fabs(acceleration * axisRes_), (axisNo_ + 1));
  //status = pC_->writeController();

  sprintf(pC_->outString_, "%i nabort", (axisNo_ + 1));
  status = pC_->writeController();
  return status;
}

asynStatus SMCpegasusAxis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "SMCpegasusAxis::setPosition";

  // The argument to the setnpos command is the distance from the current position of the
  // desired origin, which is why the position needs to be multiplied by -1.0
  sprintf(pC_->outString_, "%f %i setnpos", (position * axisRes_ * -1.0), (axisNo_ + 1));
  status = pC_->writeController();
  return status;
}

asynStatus SMCpegasusAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynSuccess;
  int regulatorMode;
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
  int done;
  int driveOn;
  int lowLimit;
  int highLimit;
  int ignoreLowLimit;
  int ignoreHighLimit;
  int axisStatus=-1;
  double position=0.0;
  asynStatus comStatus;

  static const char *functionName = "SMCpegasusAxis::poll";

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

  /*// Read the commanded velocity and acceleration
  sprintf(pC_->outString_, "%i gnv", (axisNo_ + 1));
  comStatus = pC_->writeReadController();
  
  sprintf(pC_->outString_, "%i gna", (axisNo_ + 1));
  comStatus = pC_->writeReadController();

  // Check the limit bit (0x40)
  if (axisStatus & 0x40)
  {
    asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, 
      "%s: axis %i limit indicator active.\n",
      functionName, (axisNo_ + 1));
      
    // query limits?
  }

  // Check the e-stop bit (0x80)
  if (axisStatus & 0x80)
  {
    asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, 
      "%s: axis %i emergency stopped.\n",
      functionName, (axisNo_ + 1));
  }
  
  // Check the e-stop switch active bit (0x200)
  if (axisStatus & 0x200)
  {
    asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, 
      "%s: axis %i emergency stop switch active.\n",
      functionName, (axisNo_ + 1));
    setIntegerParam(pC_->motorStatusProblem_, 1);
  }
  else{
    setIntegerParam(pC_->motorStatusProblem_, 0);
  }

  // Check the device busy bit (0x400)
  if (axisStatus & 0x400)
  {
    asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, 
      "%s: axis %i device is busy - move commands discarded.\n",
      functionName, (axisNo_ + 1));
  }*/

  // Read the limit status
  // Note: calibration switch = low limit; range measure switch = high limit
  // also need to read the switch confiruation to see if limits are ignored"
  // Don't poll the limit switches while moving, or we'll block the controller's interpreter
  if (done) {
        // Check if the axis was in homing-sequence
        if (homingInProgress_) {
            homingInProgress_ = false;
            sprintf(pC_->outString_, "2 0 %i setsw 2 1 %i setsw", (axisNo_ + 1), (axisNo_ + 1));
            pC_->writeController();
            /*setIntegerParam(pC_->motorStatusAtHome_, 1);*/
            
            // we need to wait a bit and ask the controller again about it's position
            epicsThreadSleep(0.5);
            sprintf(pC_->outString_, "%i np", (axisNo_ + 1));
            pC_->writeReadController();
            // The response string is a double
            position = atof( (char *) &pC_->inString_);
            setDoubleParam(pC_->motorPosition_, (position / axisRes_) );
            setDoubleParam(pC_->motorEncoderPosition_, (position / axisRes_) );
        }
		// Read switch confiruation
		// Bit 0:	polarity (0 = NO, 1 = NC)
		// Bit 1:	mask (0 = enabled, 1 = disabled)
		sprintf(pC_->outString_, "%i getsw", (axisNo_ + 1));
		comStatus = pC_->writeReadController();
		if (comStatus) goto skip;
		sscanf(pC_->inString_, "%i %i", &lowLimitConfig_, &highLimitConfig_);
		ignoreLowLimit = lowLimitConfig_ & 0x2;
		ignoreHighLimit = highLimitConfig_ & 0x2;
		
		// Read status of switches 0=inactive 1=active
		sprintf(pC_->outString_, "%i getswst", (axisNo_ + 1));
		comStatus = pC_->writeReadController();
		if (comStatus) goto skip;
		// The response string is of the form "0 0"
		sscanf(pC_->inString_, "%i %i", &lowLimit, &highLimit);
		//
		if (ignoreLowLimit)
			setIntegerParam(pC_->motorStatusLowLimit_, 0);
		else
			setIntegerParam(pC_->motorStatusLowLimit_, lowLimit);

		if (ignoreHighLimit)
			setIntegerParam(pC_->motorStatusHighLimit_, 0);
		else
			setIntegerParam(pC_->motorStatusHighLimit_, highLimit);
		/*setIntegerParam(pC_->motorStatusAtHome_, limit);*/

		/*// Check the drive power bit (0x100)
		driveOn = (axisStatus & 0x100) ? 0 : 1;
		setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
		setIntegerParam(pC_->motorStatusProblem_, 0);*/
  }
  
  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg SMCpegasusCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SMCpegasusCreateControllerArg1 = {"SMC pegasus port name", iocshArgString};
static const iocshArg SMCpegasusCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg SMCpegasusCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SMCpegasusCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const SMCpegasusCreateControllerArgs[] = {&SMCpegasusCreateControllerArg0,
                                                             &SMCpegasusCreateControllerArg1,
                                                             &SMCpegasusCreateControllerArg2,
                                                             &SMCpegasusCreateControllerArg3,
                                                             &SMCpegasusCreateControllerArg4};
static const iocshFuncDef SMCpegasusCreateControllerDef = {"SMCpegasusCreateController", 5, SMCpegasusCreateControllerArgs};
static void SMCpegasusCreateControllerCallFunc(const iocshArgBuf *args)
{
  SMCpegasusCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
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
