//
// Copyright (c) 2008-2017 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#pragma once
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Core/HelperThread.h>

using namespace Urho3D;
namespace Urho3D
{
class Image;
}

class CubeCapture;

//=============================================================================
//=============================================================================
URHO3D_EVENT(E_SHBUILDDONE, SHBuildDone)
{
    URHO3D_PARAM(P_NODE, Node);      // node ptr
}

//=============================================================================
//=============================================================================
class LightProbe : public StaticModel
{
    URHO3D_OBJECT(LightProbe, StaticModel);
    friend class LightProbeCreator;
public:

    LightProbe(Context* context);
    virtual ~LightProbe();
    
    static void RegisterObject(Context* context);

    void GenerateSH(const String &basepath, const String &fullpath);
    PODVector<Vector3>& GetCoeffVec() { return coeffVec_; }

    void SetDumpShCoeff(bool dump) { dumpShCoeff_ = dump; }
    void DumpSHCoeff();

protected:
    void HandleUpdate(StringHash eventType, VariantMap& eventData);
    void ForegroundProcess();
    void BackgroundProcess(void *data);
    void BeginSHBuildProcess();
    void EndSHBuild();
    void CreateThread();
    void DestroyThread();
    void CopyTextureCube();
    void ClearCoeff();
    void FinalizeCoeff();

    unsigned GetState();
    void SetState(unsigned state);
    Vector<SharedPtr<Image> >& GetCubeImages()      { return cubeImages_; }
    void SetNumSamples(int numSamples)              { numSamples_ = numSamples; }
protected:
    bool generated_;

    // sh coeff
    PODVector<Vector3> coeffVec_;
    int numSamples_;

    // cube map
    SharedPtr<CubeCapture> cubeCapture_;
    Vector<SharedPtr<Image> > cubeImages_;
    String basepath_;

    // thread
    SharedPtr<HelperThread<LightProbe> > threadProcess_;
    Mutex mutexStateLock_;

    // build state
    unsigned buildState_;

    // dbg
    bool dumpShCoeff_;

private:
    enum SHBuildType
    {
        SHBuild_Uninit,
        SHBuild_CubeCapture,
        SHBuild_BackgroundProcess,
        SHBuild_FinalizeCoeff,
        SHBuild_Complete
    };

    struct GeomData
    {
        Vector3 pos_;
        Vector3 normal_;
        Vector2 uv_;
    };
    struct SphericalData
    {
        int     x_;
        int     y_;
        int     face_;
        Vector3 normal_;
    };

    // static vars
    static PODVector<GeomData> geomData_;
    static SharedArrayPtr<unsigned short> indexBuff_;
    static unsigned numIndeces_;
    static PODVector<SphericalData> sphericalData_;
    static Mutex sphDataLock_;

    // static methods
    static void SetupUnitBoxGeom(Context *context);
    static int SetupSphericalData(const Vector<SharedPtr<Image> > &cubeImages, PODVector<Vector3> &coeffVec);
    static int CalculateSH(const Vector<SharedPtr<Image> > &cubeImages, PODVector<Vector3> &coeffVec);
    static void UpdateCoeffs(const Vector3 &vcol, const Vector3 &v, PODVector<Vector3> &coeffVec);
    static CubeMapFace GetCubefaceFromNormal(const Vector3 &normal);

    //=============================================================================
    // http://answers.unity3d.com/questions/383804/calculate-uv-coordinates-of-3d-point-on-plane-of-m.html
    // describes a barycentric calculation for 3D using proportional area calculation
    //=============================================================================
    static inline Vector3 Barycentric(const Vector2 &v0, const Vector2 &v1, const Vector2 &v2, const Vector2 &vp)
    {
        Vector3 bary(Vector3::ONE);

        // edge seg
        Vector2 e0 = v0 - vp;
        Vector2 e1 = v1 - vp;
        Vector2 e2 = v2 - vp;

        float area = CrossProduct(v1 - v0, v2 - v0);

        if (area > M_EPSILON)
        {
            // segment area: the subscripts a0, a1, and a2 are 
            // derived from the opposite subscripts of e0, e1, and e2
            float a0 = CrossProduct(e1, e2) / area;
            float a1 = CrossProduct(e2, e0) / area;
            float a2 = CrossProduct(e0, e1) / area;

            bary = Vector3(a0, a1, a2);
        }

        return bary;
    }

    // missing in Urh3D::Vector2
    static inline float CrossProduct(const Vector2 &a, const Vector2 &b)
    {
        return Abs(a.x_ * b.y_ - a.y_ * b.x_);
    }

    //=============================================================================
    // **note: another proportinal comparator - the sum of the three segment areas is equal to the 
    // area of triangle, hence you can also prove that the sum of the barycentric is equal to one, if it's inside the triangle
    //=============================================================================
    static inline bool BaryInsideTriangle(const Vector3 &bary)
    {
        return (bary.x_ + bary.y_ + bary.z_) < (1.0f + M_EPSILON);
    }
};
