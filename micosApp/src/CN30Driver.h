/*
FILENAME...   CN30Driver.h
USAGE...      Motor driver support for the Micos CN30 controller.
*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

static const char *driverName = "CN30Controller";

class epicsShareClass CN30Axis : public asynMotorAxis
{
public:
    /* These are the methods we override from the base class */
    CN30Axis(class CN30Controller *pC, int axis);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    void loopTask();

private:
    CN30Controller *pC_;          /* Pointer to the asynMotorController to which this axis belongs.
                                      Abbreviated because it is used very frequently */
    int axisIndex_;    /* Numbered from 1 */
    int axisXpos_ = 0;
    int axisYpos_ = 0;
    int steps2Go_;
    char command_, step_, raxis_, rspeed_, dir_;
    bool motorStop_;

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
