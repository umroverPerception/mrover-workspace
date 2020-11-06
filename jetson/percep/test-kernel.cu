#include <iostream>

using namespace std;

__host__ void checkStatus(cudaError_t status) {
	if (status != cudaSuccess) {
		printf("%s \n", cudaGetErrorString(status));

		return;
	}
}

__global__ void fast_add(float* a, float* b, float* res) {
	res[threadIdx.x] = a[threadIdx.x] + b[threadIdx.x];
	return;
}

int main() {
	float aHost[] = {1, 2, 3, 4, 5};
	float bHost[] = {1, 2, 3, 4, 5};
	float* aDevice, *bDevice, *resDevice, *resHost;

	cudaMalloc((void**)&aDevice, sizeof(float)*5);
	cudaMalloc((void**)&bDevice, sizeof(float)*5);
	cudaMalloc((void**)&resDevice, sizeof(float)*5);
	checkStatus(cudaGetLastError());
	resHost = (float*) malloc(sizeof(float)*5);
	cudaMemcpy(aDevice, aHost, sizeof(float)*5, cudaMemcpyHostToDevice);
	cudaMemcpy(bDevice, bHost, sizeof(float)*5, cudaMemcpyHostToDevice);
	checkStatus(cudaGetLastError());

	fast_add<<<1, 5>>>(aDevice, bDevice, resDevice);
	checkStatus(cudaGetLastError());
	cudaDeviceSynchronize();
	cudaMemcpy(resHost, resDevice, sizeof(float)*5, cudaMemcpyDeviceToHost);

	for(int i = 0; i < 5; i++) cout << resHost[i] << endl;

	free(resHost);
	cudaFree(resDevice);
	cudaFree(aDevice);
	cudaFree(bDevice);
	return 0;
}
