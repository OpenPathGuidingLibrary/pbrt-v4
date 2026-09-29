/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2013 Intel Corporation. */

#ifndef PBRT_WAVEFRONT_GUIDING_H
#define PBRT_WAVEFRONT_GUIDING_H

#ifdef PBRT_BUILD_GPU_RENDERER
#include <openpgl/gpu/OpenPGLGPU.h>
#else
#include <openpgl/cpp/OpenPGL.h>
#include <openpgl/gpu/OpenPGLGPU.h>
#endif

#include <pbrt/util/soa.h>
#include <pbrt/util/progressreporter.h>

#include <iostream>

namespace pbrt {

struct GuidedBSDFWF{
#ifdef PBRT_BUILD_GPU_RENDERER
#if defined(PBRT_IS_GPU_CODE)
    using SurfaceSamplingDistribution = openpgl::gpu::cuda::SurfaceSamplingDistribution;
    using Field = openpgl::gpu::cuda::FieldGPU;
#else
    using SurfaceSamplingDistribution = openpgl::gpu::cpu::SurfaceSamplingDistribution;
    using Field = openpgl::gpu::cpu::FieldGPU;
#endif
#else
    using SurfaceSamplingDistribution = openpgl::cpp::SurfaceSamplingDistribution;
    using Field = openpgl::cpp::Field;
#endif

    PBRT_CPU_GPU
    GuidedBSDFWF(const BSDF& bsdf, SurfaceSamplingDistribution* ssd): m_bsdf(bsdf), m_ssd(ssd) {
    }
    
    PBRT_CPU_GPU
    void Init(const Field* guiding_field, Point3f p, Float sample1D, const bool guideSurface) {
        m_enableGuiding = guideSurface;
        pgl_point3f pos = {p.x, p.y, p.z};
        bool sucess = true;
        if(m_enableGuiding && guiding_field!= nullptr && m_ssd != nullptr){
            if (IsNonSpecular(m_bsdf.Flags()) ) {
                sucess = m_ssd->Init(guiding_field, pos, sample1D);
                m_enableGuiding = sucess;
            } else {
                m_enableGuiding = false;
            }
        } else {
            m_enableGuiding = false;
        }
    }

    template<class ConcreteBxDF>
    PBRT_CPU_GPU
    SampledSpectrum f(Vector3f wo, Vector3f wi, TransportMode mode = TransportMode::Radiance) const {
        return m_bsdf.f<ConcreteBxDF>(wo, wi, mode);
    }

    template<class ConcreteBxDF>
    PBRT_CPU_GPU
    pstd::optional<BSDFSample> Sample_f(Vector3f woRender, Float u, Point2f u2,
        TransportMode mode = TransportMode::Radiance,
        BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {

        pstd::optional<BSDFSample> bs = {};
        bool sampleBSDF = true;
        if (m_enableGuiding) {
            if(m_guidingProbability > u) {
                u /= m_guidingProbability;
                sampleBSDF = false;
            } else {
                u -= m_guidingProbability;
                u /= (1.0f - m_guidingProbability);
                sampleBSDF = true;
            }
        }
        
        if (sampleBSDF){
            bs = m_bsdf.Sample_f(woRender, u, u2, mode, sampleFlags);
            if(bs && m_enableGuiding) {
                pgl_vec3f pglwi = {bs->wi[0], bs->wi[1], bs->wi[2]};
                float guidedPDF = m_ssd->PDF(pglwi);
                bs->bsdfPdf = bs->pdf;
                bs->pdf = ((1.0f - m_guidingProbability) * bs->pdf) + (m_guidingProbability * guidedPDF);
                bs->misPdf = bs->pdf;
            }
        } else {
            pgl_point2f sample2D = {u2[0], u2[1]};
            pgl_vec3f pglwi;
            float guidedPDF = m_ssd->SamplePDF(sample2D, pglwi);
            
            Vector3f wiRender = Vector3f(pglwi.x, pglwi.y, pglwi.z);
            SampledSpectrum f = m_bsdf.f(woRender, wiRender, mode);
            Float bsdfPDF = m_bsdf.PDF(woRender, wiRender);
            if(bsdfPDF > 0.f) {
                BxDFFlags flags = m_bsdf.Flags();
                Float sampledRoughness = m_bsdf.GetRoughness();
                Float eta = m_bsdf.GetEta();
                bool pdfIsProportional = false;

                float pdf = ((1.0f - m_guidingProbability) * bsdfPDF) + (m_guidingProbability * guidedPDF); 
                bs = BSDFSample(f, wiRender, pdf, flags, sampledRoughness, eta, pdfIsProportional);
                bs->bsdfPdf = bsdfPDF;
                bs->misPdf = pdf;
            }
        }
        return bs;
    }    

    template<class ConcreteBxDF>
    PBRT_CPU_GPU
    Float PDF(Vector3f woRender, Vector3f wiRender, TransportMode mode = TransportMode::Radiance,
              BxDFReflTransFlags sampleFlags = BxDFReflTransFlags::All) const {
        float bsdfPDF = m_bsdf.PDF(woRender, wiRender); 
        if (m_enableGuiding){
            float pdf = 0.f;
            pgl_vec3f pglwi = {wiRender[0], wiRender[1], wiRender[2]};
            float guidedPDF = m_ssd->PDF(pglwi);
            //if(m_guidingType == EGuideMIS) { 
                pdf = ((1.0f - m_guidingProbability) * bsdfPDF) + (m_guidingProbability * guidedPDF);
            //} else { // RIS
            //    pdf = (0.5f * bsdfPDF) + (0.5f * guidedPDF);
            //}
            return pdf;
        } else {
            return bsdfPDF;
        }
    }

    PBRT_CPU_GPU
    BxDFFlags Flags() const {
        return m_bsdf.Flags();
    }

    const BSDF& m_bsdf;
    SurfaceSamplingDistribution* m_ssd {nullptr};
    bool m_enableGuiding {false};
    float m_guidingProbability {0.5f};
};

template<class ConcretePhaseFunction>
struct GuidedPhaseFunctionWF{
#ifdef PBRT_BUILD_GPU_RENDERER
#if defined(PBRT_IS_GPU_CODE)
    using VolumeSamplingDistribution = openpgl::gpu::cuda::VolumeSamplingDistribution;
    using Field = openpgl::gpu::cuda::FieldGPU;
#else
    using VolumeSamplingDistribution = openpgl::gpu::cpu::VolumeSamplingDistribution;
    using Field = openpgl::gpu::cpu::FieldGPU;
#endif
#else
    using VolumeSamplingDistribution = openpgl::cpp::VolumeSamplingDistribution;
    using Field = openpgl::cpp::Field;
#endif
    PBRT_CPU_GPU
    GuidedPhaseFunctionWF(const ConcretePhaseFunction* phase, VolumeSamplingDistribution* vsd): m_phase(phase), m_vsd(vsd) {}

    PBRT_CPU_GPU
    void Init(const Field* guiding_field, Point3f p, Float sample1D, const bool guideVolume) {
        Float sample = -1.f;
        pgl_point3f pos = {p.x, p.y, p.z};
        m_enableGuiding = guideVolume;
        if(m_enableGuiding && guiding_field != nullptr && m_vsd != nullptr){
            m_vsd->Init(guiding_field, pos, sample);
            m_enableGuiding = true;
        } else {
            m_enableGuiding = false;
        }
    }

    PBRT_CPU_GPU
    Float p(Vector3f woRender, Vector3f wiRender) const {
        return m_phase->p(woRender, wiRender);
    }

    PBRT_CPU_GPU
    pstd::optional<PhaseFunctionSample> Sample_p(
        Vector3f woRender, Point2f u) const {

        pstd::optional<PhaseFunctionSample> ps = {};
        bool samplePhase = true;
        if (m_enableGuiding) {
            if(m_guidingProbability > u.x) {
                u.x /= m_guidingProbability;
                samplePhase = false;
            } else {
                u.x -= m_guidingProbability;
                u.x /= (1.0f - m_guidingProbability);
                samplePhase = true;
            }
        }

        if (samplePhase){
            ps = m_phase->Sample_p(woRender, u);
            if(ps && m_enableGuiding) {
                pgl_vec3f pglwi = {ps->wi[0], ps->wi[1], ps->wi[2]};
                float guidedPDF = m_vsd->PDF(pglwi);
                ps->phasePdf = ps->pdf;
                ps->pdf = ((1.0f - m_guidingProbability) * ps->pdf) + (m_guidingProbability * guidedPDF);
                ps->misPdf = ps->pdf;
            }
        } else {
            pgl_point2f sample2D = {u[0], u[1]};
            pgl_vec3f pglwi;
            float guidedPDF = m_vsd->SamplePDF(sample2D, pglwi);
            
            Vector3f wiRender = Vector3f(pglwi.x, pglwi.y, pglwi.z);
            Float p = m_phase->p(woRender, wiRender);
            Float phasePDF = m_phase->PDF(woRender, wiRender);
            if(phasePDF > 0.f) {
                Float meanCosine = m_phase->MeanCosine();
                bool pdfIsProportional = false;

                float pdf = ((1.0f - m_guidingProbability) * phasePDF) + (m_guidingProbability * guidedPDF); 
                ps = PhaseFunctionSample{p, wiRender, meanCosine, pdf, phasePDF, pdf};
            }
        }
        return ps;
        /*
        if(m_enableGuiding) {
            return m_phase->Sample_p(woRender, u);
        } else {
            return m_phase->Sample_p(woRender, u);           
        }
        */
    }

    PBRT_CPU_GPU
    float PDF(Vector3f woRender, Vector3f wiRender) const {
        float phasePDF = m_phase->PDF(woRender, wiRender); 
        if (m_enableGuiding){
            float pdf = 0.f;
            pgl_vec3f pglwi = {wiRender[0], wiRender[1], wiRender[2]};
            float guidedPDF = m_vsd->PDF(pglwi);
            //if(m_guidingType == EGuideMIS) { 
                pdf = ((1.0f - m_guidingProbability) * phasePDF) + (m_guidingProbability * guidedPDF);
            //} else { // RIS
            //    pdf = (0.5f * phasePDF) + (0.5f * guidedPDF);
            //}
            return pdf;
        } else {
            return phasePDF;
        }
    }

    const ConcretePhaseFunction* m_phase {nullptr};
    VolumeSamplingDistribution* m_vsd {nullptr};
    bool m_enableGuiding {false};
    float m_guidingProbability {0.5f};
};

#if defined(PBRT_WITH_PATH_GUIDING)
#if defined(PBRT_BUILD_GPU_RENDERER)
    using PathSegment = openpgl::gpu::cuda::PathSegment;
    using SampleData = openpgl::gpu::cuda::SampleData;
    using ZeroValueSampleData = openpgl::gpu::cuda::ZeroValueSampleData;
#else 
    using PathSegment = openpgl::gpu::cpu::PathSegment;
    using SampleData = openpgl::gpu::cpu::SampleData;
    using ZeroValueSampleData = openpgl::gpu::cpu::ZeroValueSampleData;
#endif

#if defined(PBRT_BUILD_GPU_RENDERER)
struct PathSegmentStorageBuffer: public openpgl::gpu::cuda::PathSegmentStorageBuffer {
    using Vector3 = openpgl::gpu::cuda::Vector3;
    using Point3 = openpgl::gpu::cuda::Point3;
    using Normal3 = openpgl::gpu::cuda::Normal3;
    
    PathSegmentStorageBuffer(): openpgl::gpu::cuda::PathSegmentStorageBuffer() {}

    PathSegmentStorageBuffer(int n, openpgl::gpu::Device *device): openpgl::gpu::cuda::PathSegmentStorageBuffer(n, device) {}

    PathSegmentStorageBuffer(std::string fileName, openpgl::gpu::Device *device): openpgl::gpu::cuda::PathSegmentStorageBuffer(fileName, device) {}

#else
struct PathSegmentStorageBuffer: public openpgl::gpu::cpu::PathSegmentStorageBuffer {
    using Vector3 = openpgl::gpu::cpu::Vector3;
    using Point3 = openpgl::gpu::cpu::Point3;
    using Normal3 = openpgl::gpu::cpu::Normal3;

    PathSegmentStorageBuffer(): openpgl::gpu::cpu::PathSegmentStorageBuffer() {}

    PathSegmentStorageBuffer(int n, openpgl::gpu::Device *device): openpgl::gpu::cpu::PathSegmentStorageBuffer(n, device) {}

    PathSegmentStorageBuffer(std::string fileName, openpgl::gpu::Device *device): openpgl::gpu::cpu::PathSegmentStorageBuffer(fileName, device) {}

#endif

    PBRT_CPU_GPU
    void AddSurfaceSample(const int pixelIndex, const Point3f& pos, const Normal3f& normal, const Vector3f& directionIn, const Float& pdfDirectionIn, const Vector3f& directionOut, const Vector3f& scatteringWeight, const bool isDelta, const Float roughness, const Float eta, const Float q) {
        uint32_t depth = curDepth[pixelIndex];
        if (depth < 10) {
            uint32_t idx = depth;
            segments[idx].volumeScatter[pixelIndex] = false;
            segments[idx].isDelta[pixelIndex] = isDelta;
            segments[idx].position[pixelIndex] = Point3(pos.x, pos.y, pos.z);
            segments[idx].normal[pixelIndex] = Normal3(normal.x, normal.y, normal.z);
            segments[idx].directionIn[pixelIndex] = Vector3(directionIn.x, directionIn.y, directionIn.z);
            segments[idx].pdfDirectionIn[pixelIndex] = pdfDirectionIn;
            segments[idx].directionOut[pixelIndex] = Vector3(directionOut.x, directionOut.y, directionOut.z);
            segments[idx].scatterWeight[pixelIndex] = Vector3(scatteringWeight.x, scatteringWeight.y, scatteringWeight.z);
            segments[idx].rrProbability[pixelIndex] = q;
            segments[idx].eta[pixelIndex] = eta;
            segments[idx].roughness[pixelIndex] = roughness;
            segments[idx].transmittanceWeight[pixelIndex] =  Vector3(1.f, 1.f, 1.f);
            segments[idx].scatteredContribution[pixelIndex] = Vector3(0.f, 0.f, 0.f);
            segments[idx].directContribution[pixelIndex] = Vector3(0.f, 0.f, 0.f);
            segments[idx].miWeight[pixelIndex] = 0.f;
            curDepth[pixelIndex] = depth + 1;
        }
    }

    PBRT_CPU_GPU
    void AddZeroValueSurfaceSample(const int pixelIndex, const Point3f& pos) {
        uint32_t depth = curDepth[pixelIndex];
        if (depth < 10) {
            uint32_t idx = depth;
            segments[idx].volumeScatter[pixelIndex] = false;
            segments[idx].isDelta[pixelIndex] = false;
            segments[idx].position[pixelIndex] = Point3(pos.x, pos.y, pos.z);
            segments[idx].scatterWeight[pixelIndex] = Vector3(0.f, 0.f, 0.f);
            segments[idx].scatteredContribution[pixelIndex] = Vector3(0.f, 0.f, 0.f);
            segments[idx].directContribution[pixelIndex] = Vector3(0.f, 0.f, 0.f);
            curDepth[pixelIndex] = depth + 1;
        }
    }

    PBRT_CPU_GPU
    void AddVolumeSample(const int pixelIndex, const Point3f& pos, const Normal3f& normal, const Float meanCosine) {
    }

    PBRT_CPU_GPU
    void AddInfiniteLightSample(const int pixelIndex, const Point3f& pos, const Vector3f& direction, const Vector3f& Le, const Float misWeight) {
        uint32_t depth = curDepth[pixelIndex];
        if (depth < 10) {
            uint32_t idx = depth;
            Point3f ilPos = pos + direction * 1e6f;
            segments[idx].position[pixelIndex] = Point3(ilPos.x, ilPos.y, ilPos.z);
            segments[idx].scatterWeight[pixelIndex] = Vector3(0.f, 0.f, 0.f);
            segments[idx].scatteredContribution[pixelIndex] = Vector3(0.f, 0.f, 0.f);
            segments[idx].directContribution[pixelIndex] = Vector3(Le.x, Le.y, Le.z);
            segments[idx].miWeight[pixelIndex] = misWeight;
            curDepth[pixelIndex] = depth + 1;
        }
    }

    PBRT_CPU_GPU
    void AddDirectContribution(const int pixelIndex, const Vector3f& Le, const Float misWeight) {
        uint32_t depth = curDepth[pixelIndex];
        if (depth < 10) {
            uint32_t idx = depth -1;
            segments[idx].directContribution[pixelIndex] = Vector3(Le.x, Le.y, Le.z);
            segments[idx].miWeight[pixelIndex] = misWeight;
        }
    }

    PBRT_CPU_GPU
    void AddScatteredContribution(const int pixelIndex, const Vector3f& scatteredContribution) {
        uint32_t depth = curDepth[pixelIndex];
        if (depth < 10) {
            uint32_t idx = depth -1;
            segments[idx].scatteredContribution[pixelIndex] += Vector3(scatteredContribution.x, scatteredContribution.y, scatteredContribution.z);
        }
    }
};   

#if defined(PBRT_BUILD_GPU_RENDERER)
struct SampleDataStorageBuffer: public openpgl::gpu::cuda::SampleDataStorageBuffer {
    SampleDataStorageBuffer(): openpgl::gpu::cuda::SampleDataStorageBuffer() {}

    SampleDataStorageBuffer(int n, openpgl::gpu::Device *device): openpgl::gpu::cuda::SampleDataStorageBuffer(n, device) {}

    SampleDataStorageBuffer(std::string fileName, openpgl::gpu::Device *device): openpgl::gpu::cuda::SampleDataStorageBuffer(fileName, device) {}
};
#else
struct SampleDataStorageBuffer: public openpgl::gpu::cpu::SampleDataStorageBuffer {
    SampleDataStorageBuffer(): openpgl::gpu::cpu::SampleDataStorageBuffer() {}

    SampleDataStorageBuffer(int n, openpgl::gpu::Device *device): openpgl::gpu::cpu::SampleDataStorageBuffer(n, device) {}

    SampleDataStorageBuffer(std::string fileName, openpgl::gpu::Device *device): openpgl::gpu::cpu::SampleDataStorageBuffer(fileName, device) {}
};
#endif

#endif
}

#endif
