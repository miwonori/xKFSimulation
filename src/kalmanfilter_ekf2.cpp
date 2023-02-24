// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Extended Kalman Filter
//
// ####### ANSWER FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

#include <iostream>

using namespace std;

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double GYRO_BIAS_STD = 0.001*D2R;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //

void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset) {handleLidarMeasurement(meas, map);}
}

void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the Lidar Measurements in the 
        // section below.
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
        // HINT: The mapped-matched beacon position can be accessed by the variables
        // map_beacon.x and map_beacon.y
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        // BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id

        // // cout << "LiDAR Meas id = " << meas.id << ", Beacon id = " << map_beacon.id << endl;

        // if (meas.id != -1 && map_beacon.id != -1)
        // {           
        //     // Measurement Vector
        //     VectorXd z = Vector2d::Zero();
        //     z << meas.range, meas.theta   ;

        //     // Predicted Measurement Vector (Measurement Model)
        //     VectorXd z_hat = Vector2d::Zero();
        //     double dx = map_beacon.x - state(0);
        //     double dy = map_beacon.y - state(1);
        //     double range_ = sqrt(dx*dx + dy*dy);
        //     double theta_ = wrapAngle(atan2(dy,dx) - state(2));
        //     z_hat << range_, theta_;

        //     // Measurement Model Sensitivity Matrix
        //     MatrixXd H = MatrixXd(2,5);
        //     H << -dx/range_,-dy/range_,0,0,0,
        //           dy/(range_*range_),-dx/(range_*range_),-1,0,0;

        //     // Generate Measurement Model Noise Covariance Matrix
        //     MatrixXd R = Matrix2d::Zero();
        //     R(0,0) = LIDAR_RANGE_STD*LIDAR_RANGE_STD;
        //     R(1,1) = LIDAR_THETA_STD*LIDAR_THETA_STD;

        //     VectorXd y = z - z_hat;
        //     MatrixXd S = H * cov * H.transpose() + R;
        //     MatrixXd K = cov*H.transpose()*S.inverse();
        //     MatrixXd I = MatrixXd::Identity(5,5);

        //     y(1) = wrapAngle(y(1)); // Wrap the Heading Innovation

        //     state = state + K*y;
        //     cov = (I - K*H) * cov;            
        // }
        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Prediction Step for the system in the  
        // section below.
        // HINT: Assume the state vector has the form [PX, PY, PSI, V].
        // HINT: Use the Gyroscope measurement as an input into the prediction step.
        // HINT: You can use the constants: ACCEL_STD, GYRO_STD
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        MatrixXd F = MatrixXd::Zero(5,5);
        MatrixXd Q = MatrixXd::Zero(5,5);

        F << 1,0,-dt*state(3)*sin(state(2)),dt*cos(state(2)),0,
             0,1,dt*state(3)*cos(state(2)),dt*sin(state(2)),0,
             0,0,1,0,0,
             0,0,0,1,-dt,
             0,0,0,0,1;

        Q(2,2) = dt*dt*GYRO_STD*GYRO_STD;
        Q(3,3) = dt*dt*ACCEL_STD*ACCEL_STD;
        Q(4,4) = GYRO_BIAS_STD*GYRO_BIAS_STD;

        // Update State
        double px = state(0) + dt * state(3) * cos(state(2));
        double py = state(1) + dt * state(3) * sin(state(2));
        double psi = wrapAngle(state(2) + dt * (gyro.psi_dot-state(4)));
        double v = state(3);
        double gb = state(4);
        state << px, py, psi, v, gb;
 
        cov = F * cov * F.transpose() + Q;

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    } 
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    // All this code is the same as the LKF as the measurement model is linear
    // so the UKF update state would just produce the same result.
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd(2,5);
        MatrixXd R = Matrix2d::Zero();
        MatrixXd I = MatrixXd::Identity(5,5);

        z << meas.x,meas.y;
        H << 1,0,0,0,0,
             0,1,0,0,0;
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov*H.transpose()*S.inverse();

        double GPS_threshold = 3;

        double gps_state1 = y(0)/sqrt(S(0,0));
        double gps_state2 = y(1)/sqrt(S(1,1));

        if ((fabs(gps_state1) < GPS_threshold) && (fabs(gps_state2) < GPS_threshold))
        {
            state = state + K*y;
            cov = (I - K*H) * cov;

            setState(state);
        }
        else
        {
            // cout << "GPS Measurement X = " << meas.x << ", Y = " << meas.y << endl;
            // cout << "GPS status X = " << gps_state1 << ", Y = " << gps_state2 << endl;
        }

        setCovariance(cov);
    }
    else
    {
        if (getMeasCount() >2)
        {
            VectorXd state = VectorXd::Zero(5);
            MatrixXd cov = MatrixXd::Zero(5,5);
            Vector2d PosOld = getMeasOld();
            double dx = meas.x - PosOld(0);
            double dy = meas.y - PosOld(1);

            state(0) = meas.x;
            state(1) = meas.y;
            state(2) = atan2(dy,dx);
            state(3) = sqrt(dx*dx + dy*dy);
            cov(0,0) = GPS_POS_STD*GPS_POS_STD;
            cov(1,1) = GPS_POS_STD*GPS_POS_STD;
            cov(2,2) = INIT_PSI_STD*INIT_PSI_STD;
            cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;
            setState(state);
            setCovariance(cov);
        }
        else
        {
            setMeasCount();
            setMeasOld(meas.x, meas.x);
        }
    }   
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,PSI,V,...]
        return VehicleState(state[0],state[1],state[2],state[3]);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt){}