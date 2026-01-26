#include "lib/ukf_odom.hpp"

UKF_Odom::UKF_Odom(OdomSensors *sensors, Pose *startingPose) {
    this->sensors = sensors;

    x.setZero();

    x(0) = startingPose->getX();
    x(1) = startingPose->getY();
    x(5) = startingPose->getTheta();

    computeWeights();
}

UKF_Odom::UKF_Odom(OdomSensors *sensors) 
    : UKF_Odom(sensors, new Pose()) {}

void UKF_Odom::predict(controlInputVector C) {
    X = computeSigmaPoints(x, P);

    for (int k = 0; k < numSigmaPoints; k++) {
        Y.col(k) = fx(X.col(k), C);
    }

    x = unscentedTransformStateMean(Y, wM, [this](sigmaPointStateMatrix Y, sigmaPointWeights wM) { return stateMean(Y, wM); });
    P = unscentedTransfromStateCovariance(x, Y, wC, Q, [this](stateVector s, stateVector x) { return stateResidual(s, x); });

    Y = computeSigmaPoints(x, P);
}

void UKF_Odom::update(measurementVector z) {
    for (int k = 0; k < numSigmaPoints; k++) {
        Z.col(k) = hx(Y.col(k));
    }

    zp = unscentedTransformMeasurementMean(Z, wM);
    S = unscentedTransfromMeasurementCovariance(zp, Z, wC, R);

    K = computeCrossVariance(x, zp, Y, Z, wC) * S.inverse();
    y = measurementResidual(z, zp);

    x = stateAdd(x, K*y);
    P = P - K * (S * K.transpose());
}

UKF_Odom::stateVector UKF_Odom::fx(stateVector sx, controlInputVector C) {
    stateVector sy;
    sy.setZero();

    sy(0) = sx(0) + (cos(sx(5)) * sx(2) - sin(sx(5)) * sx(3)) * dt; 
    sy(1) = sx(1) + (sin(sx(5)) * sx(2) + cos(sx(5)) * sx(3)) * dt;
    sy(2) = sx(2) + (C(0) - sx(2)) * kv * dt;
    sy(3) = sx(3) + (C(1) - sx(3)) * kv * dt;
    sy(4) = sx(4);
    sy(5) = sx(5) + (sensors->getAngularVelocity() - sx(4)) * dt;

    return sy;
}

UKF_Odom::measurementVector UKF_Odom::hx(stateVector sy) {
    measurementVector sz;
    sz.setZero();

    sz(0) = sy(0);
    sz(1) = sy(1);
    return sz;
}

UKF_Odom::stateVector UKF_Odom::unscentedTransformStateMean(sigmaPointStateMatrix Y, sigmaPointWeights wM, std::function<stateVector(sigmaPointStateMatrix, sigmaPointWeights)> meanFunc) {
    stateVector x;
    x.setZero();

    if (!meanFunc) {
        x = Y * wM;
    }
    else {
        x = meanFunc(Y, wM);
    }
    return x;
}

UKF_Odom::stateMatrix UKF_Odom::unscentedTransfromStateCovariance(stateVector x, sigmaPointStateMatrix Y, sigmaPointWeights wC, stateMatrix Q, std::function<stateVector(stateVector, stateVector)> residualFunc) {
    stateMatrix P;
    P.setZero();

    if (!residualFunc) {
        for (int k = 0; k < numSigmaPoints; k++) {
            stateVector dx = Y.col(k) - x;
            P += wC(k) * (dx * dx.transpose());
        }
    }
    else {
        for (int k = 0; k < numSigmaPoints; k++) {
            stateVector dx = residualFunc(Y.col(k), x);
            P += wC(k) * (dx * dx.transpose());
        }
    }

    P += Q;
    return P;
}

UKF_Odom::measurementVector UKF_Odom::unscentedTransformMeasurementMean(sigmaPointMeasurementMatrix Z, sigmaPointWeights wM, std::function<measurementVector(sigmaPointMeasurementMatrix, sigmaPointWeights)> meanFunc) {
    measurementVector zp;
    zp.setZero();

    if (!meanFunc) {
        zp = Z * wM;
    }
    else {
        zp = meanFunc(Z, wM);
    }
    return zp;
}

UKF_Odom::measurementMatrix UKF_Odom::unscentedTransfromMeasurementCovariance(measurementVector z, sigmaPointMeasurementMatrix Z, sigmaPointWeights wC, measurementMatrix R, std::function<measurementVector(measurementVector, measurementVector)> residualFunc) {
    measurementMatrix S;
    S.setZero();

    if (!residualFunc) {
        for (int k = 0; k < numSigmaPoints; k++) {
            measurementVector dx = Z.col(k) - z;
            S += wC(k) * (dx * dx.transpose());
        }
    }
    else {
        for (int k = 0; k < numSigmaPoints; k++) {
            measurementVector dx = residualFunc(Z.col(k), z);
            S += wC(k) * (dx * dx.transpose());
        }
    }

    S += R;
    return S;
}

UKF_Odom::sigmaPointStateMatrix UKF_Odom::computeSigmaPoints(stateVector x, stateMatrix P) {
    sigmaPointStateMatrix X;
    X.setZero();

    Eigen::Matrix<float, numStates, numStates> U = ((numStates + lambda) * P).llt().matrixL();

    X.col(0) = x;
    for (int k = 0; k < numStates; k++) {
        X.col(k+1) = x + U.col(k);
        X.col(numStates+k+1) = x - U.col(k);
    } 

    return X;
}

void UKF_Odom::computeWeights() {
    lambda = alpha * alpha * (numStates + kappa) - numStates;

    wM.fill(1 / (2 * (numStates + lambda)));
    wC.fill(wM(0));

    wM(0) = lambda / (numStates + lambda);
    wC(0) = (lambda / (numStates + lambda)) + (1 - alpha*alpha + beta);    
}

UKF_Odom::stateVector UKF_Odom::stateMean(sigmaPointStateMatrix Y, sigmaPointWeights wM) {
    stateVector x;
    x.setZero();

    float sumCos = 0.0f;
    float sumSin = 0.0f;

    for (int j = 0; j < numSigmaPoints; j++) {
        stateVector s = Y.col(j);

        for (int k = 0; k < numStates - 1; k++) {
            x(k) += s(k) * wM(j);
        }
        
        sumCos += wM(j) * cos(s(5));
        sumSin += wM(j) * sin(s(5));
    }

    x(5) = atan2(sumSin, sumCos);
    return x;
}

UKF_Odom::stateVector UKF_Odom::stateResidual(stateVector s, stateVector x) {
    stateVector y;
    y.setZero();

    y = s - x;
    y(5) = normalizeAngle(y(5));

    return y;
}

UKF_Odom::measurementVector UKF_Odom::measurementResidual(measurementVector s, measurementVector zp) {
    return s - zp;
}


UKF_Odom::stateVector UKF_Odom::stateAdd(stateVector a, stateVector b) {
    stateVector sum = a + b;
    sum(5) = atan2(sin(sum(5)), cos(sum(5)));
    return sum;
}


UKF_Odom::kalmanGain UKF_Odom::computeCrossVariance(stateVector x, measurementVector zp, sigmaPointStateMatrix Y, sigmaPointMeasurementMatrix Z, sigmaPointWeights wC) {
    kalmanGain Pxz;
    Pxz.setZero();

    for (int k = 0; k < numSigmaPoints; k++) {
        stateVector dx = stateResidual(Y.col(k), x);
        measurementVector dz = measurementResidual(Z.col(k), zp);
        Pxz += wC(k) * (dx * dz.transpose());
    }

    return Pxz;
}

float UKF_Odom::normalizeAngle(float theta) {
    float normAngle = fmod(theta, M_PI * 2);

    return normAngle > M_PI ? normAngle - (M_PI * 2) : normAngle;
}
