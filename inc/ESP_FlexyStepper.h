/*
      ******************************************************************
      *                                                                *
      *                    ESP-FlexyStepper                            *
      *                                                                *
      *            Paul Kerspe                     4.6.2020            *
      *       based on the concept of FlexyStepper by Stan Reifel      *
      *                                                                *
      ******************************************************************

 MIT License

 Copyright (c) 2020 Paul Kerspe
 Copyright (c) 2021 Pavel Nadein

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is furnished
 to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 This library is used to control one or more stepper motors.	It requires a
 stepper driver board that has a Step and Direction interface. The motors are
 accelerated and decelerated as they travel to the final position. This driver
 supports changing the target position, speed or rate of acceleration while a
 motion is in progress.

 for more details and a manual on how to use it, check the README.md on github
 and the provided examples
 https://github.com/pkerspe/ESP-FlexyStepper/blob/master/README.md

 This library is based on the works of Stan Reifel in his FlexyStepper library:
 https://github.com/Stan-Reifel/FlexyStepper
*/

#ifndef ESP_FlexyStepper_h
#define ESP_FlexyStepper_h

/* STD */
#include <stdint.h>
#include <stdbool.h>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* GPIO */
#include "driver/gpio.h"

typedef void (*callbackFunction)(void);
typedef void (*positionCallbackFunction)(int32_t);

class ESP_FlexyStepper
{
public:
	ESP_FlexyStepper();
	~ESP_FlexyStepper();
	//service functions
	bool startAsService(int coreNumber = 1);
	void stopService(void);
	bool isStartedAsService(void);

	//IO setup and helper / debugging functions
	void connectToPins(gpio_num_t stepPinNumber, gpio_num_t directionPinNumber=GPIO_NUM_NC);
	void setBrakePin(gpio_num_t brakePin, uint8_t activeState = ESP_FlexyStepper::ACTIVE_HIGH);
	uint32_t getTaskStackHighWaterMark(void);
	void clearLimitSwitchActive(void);
	bool motionComplete();
	int getDirectionOfMotion(void);
	bool isMovingTowardsHome(void);
	void emergencyStop(bool holdUntilReleased = false);
	void releaseEmergencyStop(void);
	void activateBrake(void);
	void deactivateBrake(void);
	bool isBakeActive(void);
	//the central function to calculate the next movment step signal
	bool processMovement(void);

	//register function for callbacks
	void registerHomeReachedCallback(callbackFunction homeReachedCallbackFunction);
	void registerLimitReachedCallback(callbackFunction limitSwitchTriggerdCallbackFunction);
	void registerTargetPositionReachedCallback(positionCallbackFunction targetPositionReachedCallbackFunction);
	void registerEmergencyStopTriggeredCallback(callbackFunction emergencyStopTriggerdCallbackFunction);
	void registerEmergencyStopReleasedCallback(callbackFunction emergencyStopReleasedCallbackFunction);

	//configuration functions
	void setStepsPerMillimeter(float motorStepPerMillimeter);
	void setStepsPerRevolution(float motorStepPerRevolution);
	void setSpeedInStepsPerSecond(float speedInStepsPerSecond);
	void setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond);
	void setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond);
	void setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond);
	void setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond);
	void setDecelerationInMillimetersPerSecondPerSecond(float decelerationInMillimetersPerSecondPerSecond);
	void setDecelerationInRevolutionsPerSecondPerSecond(float decelerationInRevolutionsPerSecondPerSecond);
	void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond);
	void setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond);
	void setDirectionToHome(int8_t directionTowardHome);
	void setLimitSwitchActive(int8_t limitSwitchType);

	void setBrakeEngageDelayMs(uint32_t);
	void setBrakeReleaseDelayMs(int32_t);

	float getCurrentVelocityInStepsPerSecond();
	float getCurrentVelocityInRevolutionsPerSecond();
	float getCurrentVelocityInMillimetersPerSecond(void);

	//positioning functions
	void setCurrentPositionInSteps(int32_t currentPositionInSteps);
	void setCurrentPositionInMillimeters(float currentPositionInMillimeters);
	void setCurrentPositionInRevolutions(float currentPositionInRevolutions);

	int32_t getCurrentPositionInSteps();
	float getCurrentPositionInRevolutions();
	float getCurrentPositionInMillimeters();

	void startJogging(int8_t direction);
	void stopJogging();
	void goToLimitAndSetAsHome(callbackFunction callbackFunctionForHome = NULL, int32_t maxDistanceToMoveInSteps = 2000000000L);
	void goToLimit(int8_t direction, callbackFunction callbackFunctionForLimit = NULL);
	

	void setCurrentPositionAsHomeAndStop(void);
	void setTargetPositionToStop();
	int32_t getDistanceToTargetSigned(void);

	void setTargetPositionInSteps(int32_t absolutePositionToMoveToInSteps);
	void setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
	void setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
	void setTargetPositionRelativeInSteps(int32_t distanceToMoveInSteps);
	void setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters);
	void setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions);

	int32_t getTargetPositionInSteps();
	float getTargetPositionInMillimeters();
	float getTargetPositionInRevolutions();

	//blocking function calls
	void moveToPositionInSteps(int32_t absolutePositionToMoveToInSteps);
	void moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
	void moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
	void moveRelativeInSteps(int32_t distanceToMoveInSteps);
	void moveRelativeInMillimeters(float distanceToMoveInMillimeters);
	void moveRelativeInRevolutions(float distanceToMoveInRevolutions);

	bool moveToHomeInSteps(int8_t directionTowardHome, float speedInStepsPerSecond, int32_t maxDistanceToMoveInSteps, gpio_num_t homeSwitchPin);
	bool moveToHomeInMillimeters(int8_t directionTowardHome, float speedInMillimetersPerSecond, int32_t maxDistanceToMoveInMillimeters, gpio_num_t homeLimitSwitchPin);
	bool moveToHomeInRevolutions(int8_t directionTowardHome, float speedInRevolutionsPerSecond, int32_t maxDistanceToMoveInRevolutions, gpio_num_t homeLimitSwitchPin);

	static const int8_t LIMIT_SWITCH_BEGIN = -1;
	static const int8_t LIMIT_SWITCH_END = 1;
	static const int8_t LIMIT_SWITCH_COMBINED_BEGIN_AND_END = 2;
	static const int8_t ACTIVE_HIGH = 1;
	static const int8_t ACTIVE_LOW = 2;

private:
	callbackFunction _homeReachedCallback = NULL;
	callbackFunction _limitTriggeredCallback = NULL;
	callbackFunction _emergencyStopTriggeredCallback = NULL;
	callbackFunction _emergencyStopReleasedCallback = NULL;
	positionCallbackFunction _targetPositionReachedCallback = NULL;
	callbackFunction _callbackFunctionForGoToLimit = NULL;

	static void taskRunner(void *parameter);

	void DeterminePeriodOfNextStep();
	void triggerBrakeIfNeededOrSetTimeout(void);

	gpio_num_t stepPin;
	gpio_num_t brakePin = GPIO_NUM_NC;
	uint8_t brakePinActiveState = ACTIVE_HIGH;
	uint32_t _brakeEngageDelayMs = 0;
	int32_t _brakeReleaseDelayMs = -1;
	uint32_t _timeToEngangeBrake = UINT32_MAX;
	uint32_t _timeToReleaseBrake = UINT32_MAX;
	bool _isBrakeConfigured = false;
	bool _hasMovementOccuredSinceLastBrakeRelease = true;

	gpio_num_t directionPin;
	bool _isBrakeActive = false;
	float stepsPerMillimeter;
	float stepsPerRevolution;
	int directionOfMotion;
	int32_t currentPosition_InSteps;
	int32_t targetPosition_InSteps;
	float desiredSpeed_InStepsPerSecond;
	float desiredPeriod_InUSPerStep;
	float acceleration_InStepsPerSecondPerSecond;
	float acceleration_InStepsPerUSPerUS;
	float deceleration_InStepsPerSecondPerSecond;
	float deceleration_InStepsPerUSPerUS;
	float periodOfSlowestStep_InUS;
	float minimumPeriodForAStoppedMotion;
	float nextStepPeriod_InUS;
	uint32_t lastStepTime_InUS;
	float currentStepPeriod_InUS;
	bool emergencyStopActive;
	bool holdEmergencyStopUntilExplicitRelease;
	int8_t directionTowardsHome;
	int8_t lastStepDirectionBeforeLimitSwitchTrigger;
	//true if the current stepper positon equals the homing position
	bool isCurrentlyHomed;
	bool isOnWayToHome = false;
	bool isOnWayToLimit = false;
	bool firstProcessingAfterTargetReached = true;
	//The type ID of the limit switch type that is active. possible values are LIMIT_SWITCH_BEGIN (-1) or LIMIT_SWITCH_END (1) or LIMIT_SWITCH_COMBINED_BEGIN_AND_END (2) or 0 if no limit switch is active
	int8_t activeLimitSwitch;
	bool limitSwitchCheckPeformed;
	// 0 if the the stepper is allowed to move in both directions (e.g. no limit or homing switch triggered), otherwise indicated which direction is currently not allowed for further movement
	int8_t disallowedDirection;

	TaskHandle_t xHandle = NULL;
};

// ------------------------------------ End ---------------------------------
#endif /* ESP_FlexyStepper_h */
