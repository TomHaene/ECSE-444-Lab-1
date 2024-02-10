#include "arm_math.h"


void calculateDifference(float* inputArray, float* outputArray, float* differenceArray, int size){
	for (int i = 0; i < size; i++) {
	        differenceArray[i] = inputArray[i] - outputArray[i];
	    }

}

void calculateStats(const float* differenceArray, int size, float *average, float *stdDev) {
    float sum = 0.0, variance = 0.0;

    // Calculate average
    for (int i = 0; i < size; ++i) {
    	sum += differenceArray[i];
    }
    *average = sum / size;

    // Calculate variance
    for (int i = 0; i < size; ++i) {
        variance += (differenceArray[i] - *average) * (differenceArray[i] - *average);
    }
    variance /= size;

    // Calculate standard deviation
    *stdDev = sqrt(variance);
}


float calculateCorrelation(const float* x, const float* y, int size) {
    float sumX = 0.0, sumY = 0.0, sumXY = 0.0;
    float squareSumX = 0.0, squareSumY = 0.0;

    for (int i = 0; i < size; ++i) {
        // Sum of elements of array X.
        sumX = sumX + x[i];

        // Sum of elements of array Y.
        sumY = sumY + y[i];

        // Sum of X[i] * Y[i].
        sumXY = sumXY + x[i] * y[i];

        // Sum of square of array elements.
        squareSumX = squareSumX + x[i] * x[i];
        squareSumY = squareSumY + y[i] * y[i];
    }

    // Use correlation formula
    float corr = (size * sumXY - sumX * sumY) /
                 (sqrt((size * squareSumX - sumX * sumX) * (size * squareSumY - sumY * sumY)));
    return corr;
}


void convolution(const float* x, const float* h, int N, float result[]) {
    int totalLength = 2 * N - 1; // Length of the convolution result for vectors of the same size

    // Initialize result array
    for (int i = 0; i < totalLength; ++i) {
        result[i] = 0;
    }

    // Compute the convolution
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) { // Use N again since both vectors are the same length
            result[i + j] += x[i] * h[j];
        }
    }
}

//DSP stats------------------------------------

void compute_error_dsp(float *reference_lst, float *tracked_lst, float *error, size_t length)
{
  arm_sub_f32(reference_lst, tracked_lst, error, length);
}

float compute_mean_dsp(float *array, size_t length)
{
  float mean;
  arm_mean_f32(array, length, &mean);
  return mean;
}


float compute_stddev_dsp(float *array, size_t length, int is_sample)
{
  float stddev;
  arm_std_f32(array, length, &stddev);

  // Adjust for sample standard deviation if needed
  if (is_sample && length > 1)
  {
    stddev *= sqrt((float)length / (length - 1));
  }

  return stddev;
}

float compute_correlation_dsp(float *x, float *y, int size)
{
  float sumX = 0.0f, sumY = 0.0f, sumXY = 0.0f;
  float squareSumX = 0.0f, squareSumY = 0.0f;
  float dotResult, sqrtResult;

  // Sum of elements of arrays X and Y, and their squares
  for (int i = 0; i < size; ++i)
  {
    sumX += x[i];
    sumY += y[i];
    squareSumX += x[i] * x[i];
    squareSumY += y[i] * y[i];
  }

  // Dot product of X and Y for sumXY
  arm_dot_prod_f32(x, y, size, &sumXY);

  // Calculating the correlation coefficient
  float numerator = size * sumXY - sumX * sumY;
  float denominator;

  arm_sqrt_f32((size * squareSumX - sumX * sumX) * (size * squareSumY - sumY * sumY), &sqrtResult);
  denominator = sqrtResult;

  if (denominator == 0.0f)
  {
    return 0; // Handle division by zero if necessary
  }

  float corr = numerator / denominator;
  return corr;
}

void compute_convolution_dsp(const float *x, int N, const float *h, int M, float *result)
{
  int totalLength = N + M - 1; // Length of the convolution result is correctly calculated as N+M-1

  // Ensure result is zero-initialized if not already done so by the caller
  for (int i = 0; i < totalLength; ++i)
  {
    result[i] = 0.0f;
  }

  // Perform convolution
  arm_conv_f32(x, N, h, M, result);
}



//DSP stats------------------------------------
