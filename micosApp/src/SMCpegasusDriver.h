/*
FILENAME...   SMCpegasusDriver.h
USAGE...      Motor driver support for the Micos SMC pegasus controller.

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_SMCPEGASUS_AXES 6

// Controller-specific parameters
#define NUM_SMCPEGASUS_PARAMS 0

/** drvInfo strings for extra parameters that the SMC Hydra controller supports */
//#define SMCpegasusRegulatorModeString "SMCPEGASUS_REGULATOR_MODE"

class epicsShareClass SMCpegasusAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  SMCpegasusAxis(class SMCpegasusController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus changeResolution(double newResolution);

private:
  SMCpegasusController *pC_;          /* Pointer to the asynMotorController to which this axis belongs.
                                      Abbreviated because it is used very frequently */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  int motorForm_;
  double pitch_;
  int polePairs_;
  double clPeriod_;
  double axisRes_;
  double mres_;
  int lowLimitConfig_;
  int highLimitConfig_;
  double posTravelLimit_;
  double negTravelLimit_;
  bool homingInProgress_;
  
friend class SMCpegasusController;
};

class epicsShareClass SMCpegasusController : public asynMotorController {
public:
  SMCpegasusController(const char *portName, const char *SMCpegasusPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  SMCpegasusAxis* getAxis(asynUser *pasynUser);
  SMCpegasusAxis* getAxis(int axisNo);
  asynStatus changeResolution(int axisNo, double newResolution);

//protected:
//  int SMCpegasusRegulatorMode_;    /** Regulator mode parameter index */

friend class SMCpegasusAxis;
};
