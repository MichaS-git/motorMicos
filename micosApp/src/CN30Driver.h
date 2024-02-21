/*
FILENAME...   CN30Driver.h
USAGE...      Motor driver support for the Micos CN30 controller.
*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

//static const char *driverName = "CN30Controller";

class epicsShareClass CN30Axis : public asynMotorAxis
{
public:
    /* These are the methods we override from the base class */
    CN30Axis(class CN30Controller *pC, int axis);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    void loopTask();

private:
    CN30Controller *pC_;          /* Pointer to the asynMotorController to which this axis belongs.
                                      Abbreviated because it is used very frequently */
    char axisIndex_;    /* 0x00 = axis 1 ; 0x40 = axis 2 ; 0x80 = axis 3 */
    char rspeed_;       /* Speed delay: 0x00 = 0.8ms ; 0x10 = 1.6ms ; 0x20 = 3.2ms ; 0x30 = 6.4ms */
    char dir_;          /* 0x00 = positive ; 0x08 = negative */
    char step_;         /* number of steps, 0x01 to 0x07: 1 step ... 2;5;10;20;50;100 steps*/
    char command_;      /* concatenated byte-command */
    int axisPos_;
    int steps2Go_;
    int axisDone_;
    bool axisStop_;

    friend class CN30Controller;
};

class epicsShareClass CN30Controller : public asynMotorController
{
public:
    CN30Controller(const char *portName, const char *CN30PortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

    /* These are the methods that we override from asynMotorDriver */
    asynStatus writeReadController();
    asynStatus writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout);

    friend class CN30Axis;
};
