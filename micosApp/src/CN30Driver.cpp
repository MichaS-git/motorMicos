/*
FILENAME... CN30Driver.cpp
USAGE...    Motor driver support for the Micos CN30 controller.
*/

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstdlib>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "CN30Driver.h"

/** Creates a new CN30Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] CN30PortName      The name of the drvAsynSerialPort that was created previously to connect to the CN30 controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
CN30Controller::CN30Controller(const char *portName, const char *CN30PortName, int numAxes,
                               double movingPollPeriod, double idlePollPeriod)
    :  asynMotorController(portName, numAxes,
                           0, // No controller specific parameters
                           0, // No additional interfaces beyond those in base class
                           0, // No additional callback interfaces beyond those in base class
                           ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                           1, // autoconnect
                           0, 0)  // Default priority and stack size
{
    int axis;
    asynStatus status;
    CN30Axis *pAxis;
    static const char *functionName = "CN30Controller::CN30Controller";


    /* Connect to CN30 controller */
    status = pasynOctetSyncIO->connect(CN30PortName, 0, &pasynUserController_, NULL);
    if (status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect to CN30 controller\n",
                  functionName);
    }

    for (axis=0; axis<numAxes; axis++)
    {
        pAxis = new CN30Axis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new CN30Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] CN30PortName      The name of the drvAsynIPPPort that was created previously to connect to the CN30 controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int CN30CreateController(const char *portName, const char *CN30PortName, int numAxes,
                                    int movingPollPeriod, int idlePollPeriod)
{
    CN30Controller *pCN30Controller
        = new CN30Controller(portName, CN30PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    pCN30Controller = NULL;
    return(asynSuccess);
}

/** Writes a string to the controller and reads the response.
  * Calls writeReadController() with default locations of the input and output strings
  * and default timeout. */
asynStatus CN30Controller::writeReadController()
{
    size_t nread;
    return writeReadController(outString_, inString_, sizeof(inString_), &nread, DEFAULT_CONTROLLER_TIMEOUT);
}

/** Writes a string to the controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus CN30Controller::writeReadController(const char *output, char *input,
        size_t maxChars, size_t *nread, double timeout)
{
    size_t nwrite;
    asynStatus status;
    int eomReason;
    // const char *functionName="writeReadController";

    // we only expect one byte as an answer
    maxChars = 1;

    status = pasynOctetSyncIO->writeRead(pasynUserController_, output,
                                         strlen(output), input, maxChars, timeout,
                                         &nwrite, nread, &eomReason);

    return status;
}

// These are the CN30Axis methods

/** Creates a new CN30Axis object.
  * \param[in] pC Pointer to the CN30Controller to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
CN30Axis::CN30Axis(CN30Controller *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC)
{
    //static const char *functionName = "CN30Axis::CN30Axis";

    // controller axes are numbered from 1
    axisIndex_ = axisNo + 1;

    callParamCallbacks();

}

asynStatus CN30Axis::move(double position, int relative, double baseVelocity, double slewVelocity, double acceleration)
{
    asynStatus status;
    int steps, steps2Go, moveSize;
    char command, step, raxis, rspeed, dir;
    // static const char *functionName = "CN30Axis::move";

    //chose the axis bit, we accept only two axes so far
    if (axisIndex_ == 1) {
        moveSize = position - axisXpos_;
        raxis = 0x00;
    } else {
        moveSize = position - axisYpos_;
        raxis = 0x40;
    }

    //we can choose out of 4 velocities
    if (slewVelocity == 0) rspeed = 0x00;
    else if (slewVelocity > 0 && slewVelocity <=1) rspeed =0x11;
    else if (slewVelocity > 1 && slewVelocity <=2) rspeed =0x10;
    else if (slewVelocity > 2 && slewVelocity <=3) rspeed =0x01;
    else rspeed=0x00;

    //direction and move size depend on the steps already done
    if (moveSize > 0) dir=0x00;
    else dir=0x08;

    steps2Go = abs(moveSize);

    setIntegerParam(pC_->motorStatusDone_, 0);

    while (steps2Go > 0)
    {

        if (steps2Go >= 100) step=0x07, steps = 100;
        else if (steps2Go >=  50) step=0x06, steps = 50;
        else if (steps2Go >=  20) step=0x05, steps = 20;
        else if (steps2Go >=  10) step=0x04, steps = 10;
        else if (steps2Go >=   5) step=0x03, steps = 5;
        else if (steps2Go >=   2) step=0x02, steps = 2;
        else if (steps2Go >=   1) step=0x01, steps = 1;

        command = raxis + rspeed + dir + step;
        // ex : 0x00   +  0x10   + 0x08 +   7
        // ie:  axis1    speed4  + neg  + steps
        pC_->outString_[0]=command;
        pC_->outString_[1]=0;
        pC_->writeReadController();

        steps2Go = steps2Go - steps;
    }

    if (axisIndex_ == 1) {
        axisXpos_ = position;
        setDoubleParam(pC_->motorPosition_, axisXpos_);
    } else {
        axisYpos_ = position;
        setDoubleParam(pC_->motorPosition_, axisYpos_);
    }

    setIntegerParam(pC_->motorStatusDone_, 1);

    return status;
}

asynStatus CN30Axis::stop(double acceleration)
{
    asynStatus status;
    //static const char *functionName = "CN30Axis::stop";

    //std::cout << acceleration << " acceleration\n";
    //std::cout << pC_->motorStop_ << " pC_->motorStop_\n";

    return status;
}

asynStatus CN30Axis::home(double baseVelocity, double slewVelocity, double acceleration, int forwards)
{
    asynStatus status;
    // static const char *functionName = "CN30Axis::home";

    /*status = sendAccelAndVelocity(acceleration, slewVelocity);

    // the UPR-1 60 F AIR rotation axis has 7 polepairs, the UPM-160 linear axis has 50
    // homing to the rm-switch blocks the controller-communication, so we always home to the cal-switch

    if (polePairs_ == 7) {
      sprintf(pC_->outString_, "1 0 %i setsw 1 1 %i setsw 1 0 %i sncal", (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1));
      rotHomeInProgress_ = true;
    } else {
      sprintf(pC_->outString_, "0 0 %i setsw 0 1 %i setsw 0 0 %i sncal 0.1 2 %i setncalvel", (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1), (axisNo_ + 1));
      linHomeInProgress_ = true;
    }

    status = pC_->writeController();*/
    return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status,
  * and the drive power-on status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus CN30Axis::poll(bool *moving)
{
    int done=1;
    //double position=0.0;
    asynStatus comStatus;

    //static const char *functionName = "CN30Axis::poll";

    //std::cout << stop_ << " stop_\n";

    // Read the current motor position
    /*sprintf(pC_->outString_, "%i np", (axisNo_ + 1));
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    // The response string is a double
    position = atof( (char *) &pC_->inString_);
    setDoubleParam(pC_->motorPosition_, (position / axisRes_) );
    setDoubleParam(pC_->motorEncoderPosition_, (position / axisRes_) );*/

    // Read the status of this motor
    /*sprintf(pC_->outString_, "%i nst", (axisNo_ + 1));
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    // The response string is an int
    axisStatus = atoi( (char *) &pC_->inString_);
    // Check the moving bit
    done = !(axisStatus & 0x1);
    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusMoving_, !done);
    *moving = done ? false:true;*/

    if (done)
    {
        setIntegerParam(pC_->motorStatusDone_, done);
        setIntegerParam(pC_->motorStatusMoving_, !done);
    }

    callParamCallbacks();
    return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg CN30CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg CN30CreateControllerArg1 = {"CN30 port name", iocshArgString};
static const iocshArg CN30CreateControllerArg2 = {"The last axis to count to", iocshArgInt};
static const iocshArg CN30CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg CN30CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const CN30CreateControllerArgs[] = {&CN30CreateControllerArg0,
                                                            &CN30CreateControllerArg1,
                                                            &CN30CreateControllerArg2,
                                                            &CN30CreateControllerArg3,
                                                            &CN30CreateControllerArg4,
                                                           };
static const iocshFuncDef CN30CreateControllerDef = {"CN30CreateController", 5, CN30CreateControllerArgs};
static void CN30CreateControllerCallFunc(const iocshArgBuf *args)
{
    CN30CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void CN30Register(void)
{
    iocshRegister(&CN30CreateControllerDef, CN30CreateControllerCallFunc);
}

extern "C" {
    epicsExportRegistrar(CN30Register);
}
