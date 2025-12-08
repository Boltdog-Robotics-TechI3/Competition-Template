//imu class

#include "main.h"

class Imu {
    private:
        pros::Imu imu;

    public:

	//constants
	static const int num_test_vals = 5;

    //constructor
    Imu(int port) : imu(port) {}

    float medianFilter(){
    	/*Uses a median filter to account for noise
    	in the gyro readings.
    	Gets 4 readings in quick succession and
    	then returns the median value in degrees.
    	For more information on median filters, see
    	https://en.wikipedia.org/wiki/Median_filter*/

    	//get 4 data points with 20ms delay
    	//yaw should be in degrees
    	float* yaws = get_test_vals();

    	//sort array in order
    	std::sort(yaws, yaws + 5);
        
    	//take median
    	float median_yaw = yaws[2];
    	//master.print(1, 0, "Median yaw: %f\n", median_yaw);
    	return median_yaw;

    }

    float get_mean(float data[]){
    	/*Gets the average of a list of values*/
    	float sum = 0;
    	int pop_size = sizeof(data) / sizeof(float);

    	for (int i = 0; i < pop_size; i++){
    		sum += data[i];
    	}

    	return sum / pop_size;
    }

    float get_std_dev(float data[]){
    	/*returns the standard deviation of a list of floats*/
    	int pop_size = sizeof(data) / sizeof(float);

    	float variances[pop_size];

    	float mean = get_mean(data);

    	for (int i = 0; i < pop_size; i++){
    		variances[i] = pow((data[i] - mean), 2);
    	}

    	float total_variance = get_mean(variances);

    	return sqrt(total_variance);
    }

    float* get_test_vals(){

            float* yaws = new float[num_test_vals];

        	for (int i = 0; i < num_test_vals; i++){
    		yaws[i] = imu.get_yaw();
    		//master.print(0, 0, "%f ", yaws[i]);
    		pros::delay(10);
    	}

        return yaws;
    }

};
