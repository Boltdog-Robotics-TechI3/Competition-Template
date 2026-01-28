#pragma once
#include <string>
#include <array>
#include <functional>
#include <iostream>
#include "Eigen/Eigen"
#include "odom_sensors.hpp"
#include "util/pose.hpp"

// EVERYTHING IS IN INCHES AND RADIANS
class UKF_Odom {
    private:
        OdomSensors *sensors;

        // Scalars
        static constexpr int numStates = 6;
        static constexpr int numSigmaPoints = 13; // 2n + 1
        static constexpr int numMeasurements = 2;
        static constexpr int numControlInputs = 2;

        // Type aliases
        using stateVector = Eigen::Matrix<float, numStates, 1>;
        using stateMatrix = Eigen::Matrix<float, numStates, numStates>;

        using measurementVector = Eigen::Matrix<float, numMeasurements, 1>;
        using measurementMatrix = Eigen::Matrix<float, numMeasurements, numMeasurements>;

        using controlInputVector = Eigen::Matrix<float, numControlInputs, 1>;

        using sigmaPointWeights = Eigen::Matrix<float, numSigmaPoints, 1>;
        using sigmaPointStateMatrix = Eigen::Matrix<float, numStates, numSigmaPoints>;
        using sigmaPointMeasurementMatrix = Eigen::Matrix<float, numMeasurements, numSigmaPoints>;

        using kalmanGain = Eigen::Matrix<float, numStates, numMeasurements>;

        static constexpr float dt = 0.02;
        static constexpr float kv = 0.5;

        // Used for Sigma Point Calculation
        static constexpr float alpha = 0.1;
        static constexpr float beta = 2;
        static constexpr float kappa = -3;
        float lambda = 0;

  

        // Matrices

        /**
        * Column Vector representing the state of the system in the following format:
        *
        * Px: X coordinate of the robot in the world frame
        * Py: Y coordinate of the robot in the world frame
        * Vx: X velocity of the robot in the robot frame   
        * Vy: Y velocity of the robot in the robot frame           
        * Bω: Bias of the gyro's angular velocity measurement.
        * Theta: Angle of the robot.     
        * 
        * In the Gaussian space, these are the means of each Gaussian.
        */
        stateVector x;
        stateMatrix P;

        // measurementVector z;
        measurementVector zp;
        measurementMatrix R;
        measurementMatrix S;

        stateMatrix Q;

        kalmanGain K;
        measurementVector y;     
        
        sigmaPointStateMatrix X;
        sigmaPointStateMatrix Y; 
        sigmaPointMeasurementMatrix Z;
        sigmaPointWeights wM;
        sigmaPointWeights wC;

        // Methods
        stateVector fx(const stateVector& sx, const controlInputVector& C);
        measurementVector hx(const stateVector& sy);
                
        stateVector unscentedTransformStateMean(const sigmaPointStateMatrix& Y, const sigmaPointWeights& wM, std::function<stateVector(const sigmaPointStateMatrix&, const sigmaPointWeights&)> meanFunc);
        stateMatrix unscentedTransfromStateCovariance(const stateVector& x, const sigmaPointStateMatrix& Y, const sigmaPointWeights& wC, const stateMatrix& Q, std::function<stateVector(const stateVector&, const stateVector&)> residualFunc);
                
        measurementVector unscentedTransformMeasurementMean(const sigmaPointMeasurementMatrix& Z, const sigmaPointWeights& wM, std::function<measurementVector(const sigmaPointMeasurementMatrix &, const sigmaPointWeights&)> meanFunc);
        measurementMatrix unscentedTransfromMeasurementCovariance(const measurementVector& z, const sigmaPointMeasurementMatrix& Z, const sigmaPointWeights& wC, const measurementMatrix& R, std::function<measurementVector(const measurementVector&, const measurementVector&)> residualFunc);
            
        sigmaPointStateMatrix computeSigmaPoints(const stateVector& x, const stateMatrix& P);
        void computeWeights();

        stateVector stateMean(const sigmaPointStateMatrix& Y, const sigmaPointWeights& wM);
        stateVector stateResidual(const stateVector& s, const stateVector& x);

        measurementVector measurementResidual(const measurementVector& s, const measurementVector& zp);

        stateVector stateAdd(const stateVector& a, const stateVector& b);

        kalmanGain computeCrossVariance(const stateVector& x, const measurementVector& zp, const sigmaPointStateMatrix& Y, const sigmaPointMeasurementMatrix& Z, const sigmaPointWeights& wC);

        float normalizeAngle(float theta);

    public:
        UKF_Odom(OdomSensors *sensors, const Pose& startingPose);
        UKF_Odom(OdomSensors *sensors);

        void predict(const controlInputVector& C);
        void update(const measurementVector& z);

        stateVector getState() {
            return x;
        }

        void setState(stateVector states) {
            x = states;
        }

        Eigen::Matrix<float, numStates, numStates> getStateCovariances() {
            return P;
        }


};