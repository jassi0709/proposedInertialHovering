/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * position_estimator_altitude.c: Altitude-only position estimator
 */

#include "log.h"
#include "param.h"
#include "num.h"
#include "position_estimator.h"
#include "estimator_complementary.h"

#define G 9.81;

extern bool altHoldMode;
static bool firstTime = true;
static float backupEstimatedZ;
static float backupEstimatedAcc;
static float backupEstimatedAsl;

struct selfState_s {
  float estimatedZ; // The current Z estimate, has same offset as asl
  float estimatedAcc;	// Current value of only barometer's data with closed loop
  float estimatedAsl;
  float velocityZ; // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  float estAlpha;
  float velocityFactor;
  float vAccDeadband; // Vertical acceleration deadband
  float velZAlpha;   // Blending factor to avoid vertical speed to accumulate error
  float estimatedVZ;
  float aslAlpha;
};

static struct selfState_s state = {
  .estimatedZ = 0.0,
  .estimatedAcc = 0.0,
  .estimatedAsl = 0.0,
  .velocityZ = 0.0,
  .estAlpha = 0.997,//0.997,
  .velocityFactor = 4.5,//1.293125,//1.0,
  .vAccDeadband = 0.04,//0.04,
  .velZAlpha = 0.015,//0.995,
  .estimatedVZ = 0.0,
  .aslAlpha = 0.002,
};

static uint16_t iteration = 0;
#define WHEN_TO_RESET POS_UPDATE_RATE*30		//at each Xth counter iteration, the barometer's data are considered into the Z position

static void positionEstimateInternal(state_t* estimate, float asl, float dt, struct selfState_s* state);
static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state);

void positionEstimate(state_t* estimate, float asl, float dt) {
  positionEstimateInternal(estimate, asl, dt, &state);
}

void positionUpdateVelocity(float accWZ, float dt) {
  positionUpdateVelocityInternal(accWZ, dt, &state);
}

static void positionEstimateInternal(state_t* estimate, float asl, float dt, struct selfState_s* state) {
  static float prev_estimatedZ = 0;

  /*	FIXME(Jaskirat) Should I remove this counter method and use the RATE_DO_EXECUTE method like in the
   * 	stateEstimator() function? */
  if(altHoldMode){
	  if(firstTime){
		  /*	saving the estimated parameters to be restored later
		   * 	when divergence occurs.  */
		  backupEstimatedZ = state->estimatedZ;
		  backupEstimatedAcc = state->estimatedAcc;
		  backupEstimatedAsl = state->estimatedAsl;
		  firstTime = false;
	  }
	  else if(iteration>=WHEN_TO_RESET){
		  /*	Restore the backed up values if some time
		   * 	has been elapsed */
		  state->estimatedZ = backupEstimatedZ;
		  state->estimatedAcc = backupEstimatedAcc;
		  state->estimatedAsl = backupEstimatedAsl;

		  iteration = 0;
	  }

	  iteration++;
  }
  else
	  firstTime = true;

  //complementary filter for barometer's data
  state->estimatedAsl = state->estAlpha * state->estimatedAsl +
		  (state->aslAlpha) * asl;

  //complementary filter for accelerometer's data
  state->estimatedAcc = state->estAlpha * state->estimatedAcc +
		  state->velocityFactor * state->velocityZ * dt;

  //state->estimatedZ = (state->estimatedAcc + state->estimatedAsl)/2;
  state->estimatedZ = (16.5/20.0)*state->estimatedAcc;
  state->estimatedZ += (3.5/20.0)*state->estimatedAsl;


  estimate->position.x = 0.0;
  estimate->position.y = 0.0;
  estimate->position.z = state->estimatedZ;
  estimate->velocity.z = (state->estimatedZ - prev_estimatedZ) / dt;
  state->estimatedVZ = estimate->velocity.z;
  prev_estimatedZ = state->estimatedZ;
}

static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state) {
  state->velocityZ += deadband(accWZ, state->vAccDeadband) * dt * G;
  state->velocityZ *= state->velZAlpha;
}

LOG_GROUP_START(posEstimatorAlt)
LOG_ADD(LOG_FLOAT, estimatedZ, &state.estimatedZ)
LOG_ADD(LOG_FLOAT, estVZ, &state.estimatedVZ)
LOG_ADD(LOG_FLOAT, velocityZ, &state.velocityZ)
LOG_GROUP_STOP(posEstimatorAlt)

PARAM_GROUP_START(posEst)
PARAM_ADD(PARAM_FLOAT, estAlpha, &state.estAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &state.aslAlpha)
PARAM_ADD(PARAM_FLOAT, velFactor, &state.velocityFactor)
PARAM_ADD(PARAM_FLOAT, velZAlpha, &state.velZAlpha)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &state.vAccDeadband)
PARAM_GROUP_STOP(posEstimatorAlt)
