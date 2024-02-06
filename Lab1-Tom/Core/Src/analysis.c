
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

