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

    // 0x00 = axis 1 ; 0x40 = axis 2 ; 0x80 = axis 3
    if (axisNo == 0) axisIndex_ = 0x00;
    else if (axisNo == 1) axisIndex_ = 0x40;
    else if (axisNo == 2) axisIndex_ = 0x80;

    axisPos_ = 0;
    axisStop_ = false;
    axisDone_ = 1;

    callParamCallbacks();
}

static void c_looptask(void *arg)
{
  CN30Axis *p = (CN30Axis *)arg;
  p->loopTask();
}

void CN30Axis::loopTask(void)
{
    int steps;
    //static const char *functionName = "loopTask";

    axisDone_ = 0;

    while (steps2Go_ > 0) {

        if (axisStop_) {
            axisDone_ = 1;
            axisStop_ = false;
            break;
        }

        if (steps2Go_ >= 100) step_=0x07, steps = 100;
        else if (steps2Go_ >=  50) step_=0x06, steps = 50;
        else if (steps2Go_ >=  20) step_=0x05, steps = 20;
        else if (steps2Go_ >=  10) step_=0x04, steps = 10;
        else if (steps2Go_ >=   5) step_=0x03, steps = 5;
        else if (steps2Go_ >=   2) step_=0x02, steps = 2;
        else if (steps2Go_ >=   1) step_=0x01, steps = 1;

        command_ = axisIndex_ + rspeed_ + dir_ + step_;
        // ex : 0x00   +  0x10   + 0x08 +   7
        // ie:  axis1    speed4  + neg  + steps
        pC_->outString_[0]=command_;
        pC_->outString_[1]=0;
        pC_->writeReadController();

        steps2Go_ = steps2Go_ - steps;

        if (dir_) {
            axisPos_ = axisPos_ - steps;
        } else {
            axisPos_ = axisPos_ + steps;
        }
    }
    axisDone_ = 1;
}

asynStatus CN30Axis::move(double position, int relative, double baseVelocity, double slewVelocity, double acceleration)
{
    asynStatus status = asynSuccess;
    int moveSize;
    // static const char *functionName = "CN30Axis::move";

    moveSize = position - axisPos_;

    // I don't know how to read the MRES out of the parameter library correctly ...
    // that's why hard coded multiplication with MRES
    // we can choose out of 4 velocities, 0 and greater 3 means fast as possible
    if (slewVelocity * 3e-4 == 0) rspeed_ = 0x00;
    else if (slewVelocity * 3e-4 > 0 && slewVelocity * 3e-4 <=1) rspeed_ =0x30;
    else if (slewVelocity * 3e-4 > 1 && slewVelocity * 3e-4 <=2) rspeed_ =0x20;
    else if (slewVelocity * 3e-4 > 2 && slewVelocity * 3e-4 <=3) rspeed_ =0x10;
    else rspeed_=0x00;

    //direction and move size depend on the steps already done
    if (moveSize > 0) dir_=0x00;
    else dir_=0x08;

    steps2Go_ = abs(moveSize);

    /* launch the while-loop task in separate thread*/
    epicsThreadCreate("CN30AxisWhileLoopTask",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_looptask, this);

    return status;
}

asynStatus CN30Axis::stop(double acceleration)
{
    asynStatus status = asynSuccess;
    //static const char *functionName = "CN30Axis::stop";

    axisStop_ = true;

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
    asynStatus comStatus = asynSuccess;

    setDoubleParam(pC_->motorPosition_, axisPos_);

    setIntegerParam(pC_->motorStatusDone_, axisDone_);
    setIntegerParam(pC_->motorStatusMoving_, !axisDone_);
    *moving = axisDone_ ? false:true;
    axisStop_ = axisDone_ ? true:false;

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
