#pragma once
#include <string>
#include <functional>
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
        using stateVector = Eigen::Vector<float, numStates>;
        using stateMatrix = Eigen::Matrix<float, numStates, numStates>;

        using measurementVector = Eigen::Vector<float, numMeasurements>;
        using measurementMatrix = Eigen::Matrix<float, numMeasurements, numMeasurements>;

        using controlInputVector = Eigen::Vector<float, numControlInputs>;

        using sigmaPointWeights = Eigen::Vector<float, numSigmaPoints>;
        using sigmaPointStateMatrix = Eigen::Matrix<float, numStates, numSigmaPoints>;
        using sigmaPointMeasurementMatrix = Eigen::Matrix<float, numMeasurements, numSigmaPoints>;

        using kalmanGain = Eigen::Matrix<float, numStates, numMeasurements>;

        static constexpr float dt = 0.02;
        static constexpr float kv = 0.5;

        // Used for Sigma Point Calculation
        static constexpr float alpha = 0.1;
        static constexpr float beta = 2;
        static constexpr float kappa = -3;
        static float lambda;

  

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
        stateVector fx(stateVector s, controlInputVector C);
        measurementVector hx(stateVector s);
        
        stateVector unscentedTransformStateMean(sigmaPointStateMatrix Y, sigmaPointWeights wM, std::function<stateVector(sigmaPointStateMatrix, sigmaPointWeights)> meanFunc = {});
        stateMatrix unscentedTransfromStateCovariance(stateVector x, sigmaPointStateMatrix Y, sigmaPointWeights wC, stateMatrix Q, std::function<stateVector(stateVector, stateVector)> residualFunc = {});
        
        measurementVector unscentedTransformMeasurementMean(sigmaPointMeasurementMatrix Z, sigmaPointWeights wM, std::function<measurementVector(sigmaPointMeasurementMatrix, sigmaPointWeights)> meanFunc = {});
        measurementMatrix unscentedTransfromMeasurementCovariance(measurementVector z, sigmaPointMeasurementMatrix Z, sigmaPointWeights wC, measurementMatrix R, std::function<measurementVector(measurementVector, measurementVector)> residualFunc = {});
    
        sigmaPointStateMatrix computeSigmaPoints(stateVector x, stateMatrix P);
        void computeWeights();

        stateVector stateMean(sigmaPointStateMatrix Y, sigmaPointWeights wM);
        stateVector stateResidual(stateVector s, stateVector x);

        measurementVector measurementResidual(measurementVector s, measurementVector zp);

        stateVector stateAdd(stateVector a, stateVector b);

        kalmanGain computeCrossVariance(stateVector x, measurementVector zp, sigmaPointStateMatrix Y, sigmaPointMeasurementMatrix Z, sigmaPointWeights wC);

        float normalizeAngle(float theta);

    public:
        UKF_Odom(OdomSensors *sensors, Pose *startingPose);
        UKF_Odom(OdomSensors *sensors);

        void predict(controlInputVector C);
        void update(measurementVector z);

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