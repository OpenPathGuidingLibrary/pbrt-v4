#pragma once

#if defined(PBRT_WITH_PATH_GUIDING)
    #include <openpgl/cpp/OpenPGL.h>
#if defined(PBRT_BUILD_GPU_RENDERER)
    #include <openpgl/gpu/Device.h>
    #include <openpgl/gpu/OpenPGLGPU.h>
#endif // PBRT_BUILD_GPU_RENDERER
#endif // PBRT_WITH_PATH_GUIDING

namespace pbrt {

#if defined(PBRT_WITH_PATH_GUIDING)

struct PBRTGuidingOptions {
    bool update;
    bool enableGuiding;
    bool guideSurface;
    bool guideVolume;
    float surfaceGuidingProbability;
    float volumeGuidingProbability;
#ifdef PBRT_BUILD_GPU_RENDERER
#if defined(PBRT_IS_GPU_CODE)
    openpgl::gpu::cuda::FieldGPU guidingField;
#else
    openpgl::gpu::cpu::FieldGPU guidingField;
#endif // PBRT_IS_GPU_CODE
#else
    std::shared_ptr<openpgl::cpp::Field> guidingField;
#endif // PBRT_BUILD_GPU_RENDERER
};

extern PBRTGuidingOptions *GuidingOptions;
#ifdef PBRT_BUILD_GPU_RENDERER
extern __constant__ PBRTGuidingOptions GuidingOptionsGPU;
#endif // PBRT_BUILD_GPU_RENDERER
#endif // PBRT_WITH_PATH_GUIDING

#ifdef PBRT_BUILD_GPU_RENDERER
#if defined(PBRT_WITH_PATH_GUIDING)
void CopyGuidingOptionsToGPU();
#endif // PBRT_WITH_PATH_GUIDING
#endif

#if defined(PBRT_WITH_PATH_GUIDING)
// Options Inline Functions
PBRT_CPU_GPU inline const PBRTGuidingOptions &GetGuidingOptions();

PBRT_CPU_GPU inline const PBRTGuidingOptions &GetGuidingOptions() {
#if defined(PBRT_IS_GPU_CODE)
    return GuidingOptionsGPU;
#else
    return *GuidingOptions;
#endif // PBRT_IS_GPU_CODE
}
#endif // PBRT_WITH_PATH_GUIDING

}