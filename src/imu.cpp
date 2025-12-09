//imu class

#include "main.h"

class Imu {
    private:
        pros::Imu imu;

    public:

	//constants
	static const int num_test_vals = 5;
	static const int num_std_dev = 2;

	//not constants
	float mean;
	float std_dev;

    //constructor
    Imu(int port) : imu(port) {
		calibrate();
	}

	//

	//other functions
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

    	for (int i = 0; i < pop_size; i++){
    		variances[i] = pow((data[i] - mean), 2);
    	}

    	float total_variance = get_mean(variances);

    	return sqrt(total_variance);
    }

    float* get_test_vals(){
		/*Gets a number of yaw data points from the imu
		with a 10 ms delay between each, then returns 
		them as an array. Default number of points is 5
		*/

            float* yaws = new float[num_test_vals];

        	for (int i = 0; i < num_test_vals; i++){
    		yaws[i] = imu.get_yaw();
    		//master.print(0, 0, "%f ", yaws[i]);
    		pros::delay(10);
    	}

        return yaws;
    }

	void calibrate(){
		/*Calibrates IMU*/

		//call pros default reset function
		imu.reset();

		//get test vals
		float* test_values = get_test_vals();

		//calculate mean and standard deviation
		mean = get_mean(test_values);
		std_dev = get_std_dev(test_values);
	}

};
