/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/filter_base.h"
#include "robot_localization/filter_common.h"
#include "robot_localization/PingThread.h"
#include <ros/time.h>
#include <ros/duration.h>
#include <ros/console.h>

#include <sstream>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <iostream>
#include "std_msgs/String.h"

namespace RobotLocalization
{
  FilterBase::FilterBase() :
    state_(STATE_SIZE),
    predictedState_(STATE_SIZE),
    futureState_(STATE_SIZE),
    old_futureState_(STATE_SIZE),
    transferFunction_(STATE_SIZE, STATE_SIZE),
    transferFunctionJacobian_(STATE_SIZE, STATE_SIZE),
    estimateErrorCovariance_(STATE_SIZE, STATE_SIZE),
    covarianceEpsilon_(STATE_SIZE, STATE_SIZE),
    processNoiseCovariance_(STATE_SIZE, STATE_SIZE),
    identity_(STATE_SIZE, STATE_SIZE),
    debug_(false),
    debugStream_(NULL)
  {
    initialized_ = false;
    // Clear the state and predicted state
    state_.setZero();
    predictedState_.setZero();
    futureState_.setZero();
    old_futureState_.setZero();

    // Prepare the invariant parts of the transfer
    // function
    transferFunction_.setIdentity();

    // Clear the Jacobian
    transferFunctionJacobian_.setZero();

    // Set the estimate error covariance. We want our measurements
    // to be accepted rapidly when the filter starts, so we should
    // initialize the state's covariance with large values.
    estimateErrorCovariance_.setIdentity();
    estimateErrorCovariance_ *= 1e-9;

    // We need the identity for the update equations
    identity_.setIdentity();

    // Set the epsilon matrix to be a matrix with small values on the diagonal
    // It is used to maintain the positive-definite property of the covariance
    covarianceEpsilon_.setIdentity();
    covarianceEpsilon_ *= 0.001;

    // Assume 200Hz from sensor data and data often arrives at bunches (configurable)
    sensorTimeout_ = ros::Duration(0.05);

    // Initialize our last update and measurement times
    ControlTimeEnd = ros::Time::now();
    lastUpdateTime_ = ros::Time::now();
    lastMeasurementTime_ = ros::Time::now();
    lastFutureUpdateTime_ = ros::Time::now();
    lastPingTime_ = ros::Time::now();

    // These can be overridden via the launch parameters,
    // but we need default values.
    processNoiseCovariance_.setZero();
    processNoiseCovariance_(StateMemberX, StateMemberX) = 0.05;
    processNoiseCovariance_(StateMemberY, StateMemberY) = 0.05;
    processNoiseCovariance_(StateMemberZ, StateMemberZ) = 0.06;
    processNoiseCovariance_(StateMemberRoll, StateMemberRoll) = 0.03;
    processNoiseCovariance_(StateMemberPitch, StateMemberPitch) = 0.03;
    processNoiseCovariance_(StateMemberYaw, StateMemberYaw) = 0.06;
    processNoiseCovariance_(StateMemberVx, StateMemberVx) = 0.025;
    processNoiseCovariance_(StateMemberVy, StateMemberVy) = 0.025;
    processNoiseCovariance_(StateMemberVz, StateMemberVz) = 0.04;
    processNoiseCovariance_(StateMemberVroll, StateMemberVroll) = 0.01;
    processNoiseCovariance_(StateMemberVpitch, StateMemberVpitch) = 0.01;
    processNoiseCovariance_(StateMemberVyaw, StateMemberVyaw) = 0.02;
    processNoiseCovariance_(StateMemberAx, StateMemberAx) = 0.01;
    processNoiseCovariance_(StateMemberAy, StateMemberAy) = 0.01;
    processNoiseCovariance_(StateMemberAz, StateMemberAz) = 0.015;
  }

  FilterBase::~FilterBase()
  {
  }

  bool FilterBase::getDebug()
  {
    return debug_;
  }

  const Eigen::MatrixXd& FilterBase::getEstimateErrorCovariance()
  {
    return estimateErrorCovariance_;
  }

  bool FilterBase::getInitializedStatus()
  {
    return initialized_;
  }

  ros::Time FilterBase::getLastMeasurementTime()
  {
    return lastMeasurementTime_;
  }

  ros::Time FilterBase::getLastUpdateTime()
  {
    return lastUpdateTime_;
  }

  ros::Time FilterBase::getLastControlTime()
  {
      return ControlTimeEnd;
  }

  ros::Time FilterBase::getLastFutureUpdateTime()
  {
    return lastFutureUpdateTime_;
  }

  const Eigen::VectorXd& FilterBase::getPredictedState()
  {
    return predictedState_;
  }

  const Eigen::MatrixXd& FilterBase::getProcessNoiseCovariance()
  {
    return processNoiseCovariance_;
  }

  ros::Duration FilterBase::getSensorTimeout()
  {
    return sensorTimeout_;
  }

  ros::Time FilterBase::getLastPingTime()
  {
    return lastPingTime_;
  }

  const Eigen::VectorXd& FilterBase::getState()
  {
    return state_;
  }

  const Eigen::VectorXd& FilterBase::getFutureState()
  {
    return futureState_;
  }

  void FilterBase::processMeasurement(const Measurement measurement, ControlQueue &queue)
  {
    FB_DEBUG("------ FilterBase::processMeasurement (" << measurement.topicName_ << ") ------\n");
    ros::Duration preddelta = ros::Duration(0.0);
    ros::Duration predtime = ros::Duration(0.0);
    ros::Duration sum = ros::Duration(0.0);
    ros::Duration predup = ros::Duration(0.0);
    ros::Duration input_delta = ros::Duration(0.0);

    Control control_input;
    Control control_next;

    // If we've had a previous reading, then go through the predict/update
    // cycle. Otherwise, set our state and covariance to whatever we get
    // from this measurement.
    if (initialized_)
    {
      // Determine how much time has passed since our last measurement
     predtime = measurement.time_ - lastMeasurementTime_;
     FB_DEBUG("Filter is already initialized. Carrying out predict/correct loop...\n"
               "Measurement time is " << std::setprecision(20) << measurement.time_ <<
               ", last measurement time is " << lastMeasurementTime_ << ", delta is " << predtime << "\n");

      // If new measurement arrives but new and old measurement have same time stamp
      // Measurment arrives at 200Hz. Therefore we assume 0.005s time steps
      if( predtime == ros::Duration(0.0) && !queue.empty())
      {
          control_input = queue.top();
          control_next = queue.top();
          input_delta = control_next.time_ - control_input.time_;
          ros::Duration measurement_delta = zeroPredTime;
          // if several messages arrive with same time stamp we still want to move forward with control input
          while(measurement_delta > input_delta && !queue.empty())
          {
              measurement_delta = measurement_delta - input_delta;
              queue.pop();
              control_input = queue.top();
          }

          predict(0.004, control_input.control_);
          predictedState_ = state_;
          zeroPredTime = zeroPredTime + ros::Duration(0.004);
      }
      else if (predtime > ros::Duration(0.0))
      {
             zeroPredTime = ros::Duration(0.0);
             validateDelta(predtime);
             //Sums up prediction. To ensure linearized assumptions max step time is limited to 10ms
             while(predup != predtime && !queue.empty())
             {
                 control_input = queue.top();
                 queue.pop();

                 //Final call
                 if( (predtime - predup) <= ros::Duration(0.01))
                 {
                     predict((predtime - predup).toSec(), control_input.control_);
                     predup = predtime;
                 }
                 // More than 10ms forward in time to predict
                 else if((predtime - predup) > ros::Duration(0.01))
                 {
                     control_next = queue.top();
                     //calculate step size of control input and predict till new control command
                     input_delta = control_next.time_ - control_input.time_;
                     // To ensure that we do not overshoot in time
                     if(input_delta >= (predtime - predup)) input_delta = (predtime - predup);
                     // if step size smaller than 10ms
                     if(input_delta <= ros::Duration(0.01))
                     {
                         predict(input_delta.toSec(), control_input.control_);
                     }
                     //sum up till input_delta
                     else if(input_delta > ros::Duration(0.01))
                     {
                         sum = ros::Duration(0.0);
                         preddelta = ros::Duration(0.0);
                         while(sum != input_delta)
                         {
                             preddelta =  std::max(ros::Duration(0.0), std::min(input_delta -sum,ros::Duration(0.01)));
                             predict(preddelta.toSec(), control_input.control_);
                             sum = sum + preddelta;
                         }
                     }
                    predup = predup + input_delta;
               }
             }
             // Return this to the user
             predictedState_ = state_;
    }
    correct(measurement);
    }
    else
    {
      FB_DEBUG("First measurement. Initializing filter.\n");

      // Initialize the filter, but only with the values we're using
      size_t measurementLength = measurement.updateVector_.size();
      for (size_t i = 0; i < measurementLength; ++i)
      {
        state_[i] = (measurement.updateVector_[i] ? measurement.measurement_[i] : state_[i]);
      }

      // Same for covariance
      for (size_t i = 0; i < measurementLength; ++i)
      {
        for (size_t j = 0; j < measurementLength; ++j)
        {
          estimateErrorCovariance_(i, j) = (measurement.updateVector_[i] && measurement.updateVector_[j] ?
                                            measurement.covariance_(i, j) :
                                            estimateErrorCovariance_(i, j));
        }
      }

      initialized_ = true;
    }

    if (predtime >= ros::Duration(0.0))
    {
      // Update the last measurement and update time.
      // The measurement time is based on the time stamp of the
      // measurement, whereas the update time is based on this
      // node's current ROS time. The update time is used to
      // determine if we have a sensor timeout, whereas the
      // measurement time is used to calculate time deltas for
      // prediction and correction.
        lastMeasurementTime_ = measurement.time_;
    }

    FB_DEBUG("------ /FilterBase::processMeasurement (" << measurement.topicName_ << ") ------\n");
  }

  void FilterBase::futurePrediction(ros::Duration predtime, ControlQueue &queue)
  {
       Control control_input;
       Control control_next;
       ros::Duration input_delta = ros::Duration(0.0);
       ros::Duration preddelta = ros::Duration(0.0);
       ros::Duration predup = ros::Duration(0.0);

       if(queue.empty())
       {
          futureState_ = old_futureState_;
       }

       // Predict till predtime is reached
       while(predup != predtime && !queue.empty())
       {
           control_input = queue.top();
           queue.pop();
           // Predict in step size of the time gap between control inputs
           if(!queue.empty())
           {
               control_next = queue.top();
               // Calculate step size of control input and predict till new control command
               input_delta = control_next.time_ - control_input.time_;
               // To ensure we do not overshoot in time
               if(input_delta >= (predtime - predup))
               {
                   input_delta = (predtime - predup);
               }
               stepaheadPrediction(input_delta.toSec(), control_input.control_);
               predup = predup + input_delta;
           }
           else
           {
               // If no control data is avialable perform a final prediction
               preddelta =  std::max(ros::Duration(0.0), std::min(predtime-predup,ros::Duration(0.01)));
               stepaheadPrediction(input_delta.toSec(), control_input.control_);
           }
           old_futureState_ = futureState_;
       }
 }





  void FilterBase::ProcessHalfPredict(const Measurement &measurement, ControlQueue &queue)
  {
    ros::Duration predtime = ros::Duration(0.0);
    ros::Duration predup = ros::Duration(0.0);
    ros::Duration input_delta = ros::Duration(0.0);

    Control control_input;
    Control control_next;

    if (initialized_)
    {
     // Determine how much time has passed since our last measurement
     predtime = measurement.time_ - lastFutureUpdateTime_;
     validateDelta(predtime);

     // Only want to carry out a prediction if it's
     // forward in time. Otherwise, just correct.
     if(predtime == ros::Duration(0.0))
     {
         control_input = queue.top();
         stepaheadPrediction(0.05, control_input.control_);
     }
     else if (predtime > ros::Duration(0.0)) //  && predtime <= ros::Duration(0.01))
     {
         control_input = queue.top();
         stepaheadPrediction((predtime - predup).toSec(), control_input.control_);
     }
     else if ( predtime > ros::Duration(0.01))
     {
         double pop = 0;
         while(predup != predtime && !queue.empty())
         {
             control_input = queue.top();
             queue.pop();
             if(!queue.empty())
             {
                 control_next = queue.top();
                 //calculate step size of control input and predict till new control command
                 input_delta = control_next.time_ - control_input.time_;
                 // To ensure that we do not overshoot in time
                 if(input_delta >= (predtime - predup)) input_delta = (predtime - predup);
                 // Predict
                 stepaheadPrediction(input_delta.toSec(), control_input.control_);
             }
             predup = predup + input_delta;
             pop = pop + 1;
         }
     }
     lastFutureUpdateTime_ = measurement.time_;
     FutureCorrect(measurement);
    }
  }

  void FilterBase::setDebug(const bool debug, std::ostream *outStream)
  {
    if (debug)
    {
      if (outStream != NULL)
      {
        debugStream_ = outStream;
        debug_ = true;
      }
      else
      {
        debug_ = false;
      }
    }
    else
    {
      debug_ = false;
    }
  }

  void FilterBase::setEstimateErrorCovariance(const Eigen::MatrixXd &estimateErrorCovariance)
  {
    estimateErrorCovariance_ = estimateErrorCovariance;
  }

  void FilterBase::setLastMeasurementTime(ros::Time lastMeasurementTime)
  {
    lastMeasurementTime_ = lastMeasurementTime;
  }

  void FilterBase::setLastUpdateTime(ros::Time lastUpdateTime)
  {
    lastUpdateTime_ = lastUpdateTime;
  }
  void FilterBase::setLastFutureUpdateTime(ros::Time lastFutureUpdateTime)
  {
    lastFutureUpdateTime_ = lastFutureUpdateTime;
  }

  void FilterBase::setProcessNoiseCovariance(const Eigen::MatrixXd &processNoiseCovariance)
  {
    processNoiseCovariance_ = processNoiseCovariance;
  }

  void FilterBase::setSensorTimeout(ros::Duration sensorTimeout)
  {
    sensorTimeout_ = sensorTimeout;
  }

  void FilterBase::setLastPingTime(ros::Time time)
  {
    lastPingTime_ = time;
  }


  void FilterBase::setState(const Eigen::VectorXd &state)
  {
    state_ = state;
  }

  void FilterBase::setFutureState(const Eigen::VectorXd &state)
  {
      futureState_ = state;
  }

  void FilterBase::validateDelta(ros::Duration &delta)
  {
    // This handles issues with ROS time when use_sim_time is on and we're playing from bags.
    if (delta > ros::Duration(ros::Duration(1000.0)))
    {
      FB_DEBUG("Delta was very large. Suspect playing from bag file. Setting to 0.01\n");

      delta = ros::Duration(0.01);
    }
  }

  void FilterBase::wrapStateAngles()
  {
    state_(StateMemberRoll)  = FilterUtilities::clampRotation(state_(StateMemberRoll));
    state_(StateMemberPitch) = FilterUtilities::clampRotation(state_(StateMemberPitch));
    state_(StateMemberYaw)   = FilterUtilities::clampRotation(state_(StateMemberYaw));
  }

  bool FilterBase::checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
                                             const Eigen::MatrixXd &invCovariance,
                                             const double nsigmas)
  {
    double sqMahalanobis = innovation.dot(invCovariance * innovation);
    double threshold = nsigmas * nsigmas;

    if (sqMahalanobis >= threshold)
    {
      FB_DEBUG("Innovation mahalanobis distance test failed. Squared Mahalanobis is: " << sqMahalanobis << "\n" <<
               "Threshold is: " << threshold << "\n" <<
               "Innovation is: " << innovation << "\n" <<
               "Innovation covariance is:\n" << invCovariance << "\n");

      return false;
    }

    return true;
  }
}  // namespace RobotLocalization
