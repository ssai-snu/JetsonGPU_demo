#define HELPERGL_EXTERN_GL_FUNC_IMPLEMENTATION
#include <helper_gl.h>
#include <helper_cuda.h>
#include <cstdlib>
#include <cstdio>
#include <string.h>
#include <cuda_gl_interop.h>

#include "thrust/device_ptr.h"
#include "thrust/for_each.h"
#include "thrust/iterator/zip_iterator.h"
#include "thrust/sort.h"

#include "particles_kernel_device.cuh"
#include "ParticleSystem.cuh"

extern "C"
{

    cudaArray *noiseArray;

    void setParameters(SimParams *hostParams)
    {
        // copy parameters to constant memory
        // TODO 1
    }

    //Round a / b to nearest higher integer value
    int iDivUp(int a, int b)
    {
        // TODO 2
    }

    // compute grid and thread block size for a given number of elements
    void computeGridSize(int n, int blockSize, int &numBlocks, int &numThreads)
    {
        // TODO 3
    }

    inline float frand()
    {
        // TODO 4
    }

    // create 3D texture containing random values
    void createNoiseTexture(int w, int h, int d)
    {
        // TODO 5
    }

    void
    integrateSystem(float4 *oldPos, float4 *newPos,
                    float4 *oldVel, float4 *newVel,
                    float deltaTime,
                    int numParticles)
    {
        // TODO 6
    }

    void calcDepth(float4  *pos,
                    float   *keys,        // output
                    uint    *indices,     // output
                    float3   sortVector,
                    int      numParticles)
    {
        thrust::device_ptr<float4> d_pos(pos);
        thrust::device_ptr<float> d_keys(keys);
        thrust::device_ptr<uint> d_indices(indices);

        // TODO 7
    }

    void sortParticles(float *sortKeys, uint *indices, uint numParticles)
    {
        // TODO 8
    }

}   // extern "c"
