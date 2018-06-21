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

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Resource/Image.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/Log.h>
#include <cstdio>

#include "LightProbe.h"
#include "CubeCapture.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
// static vars
//=============================================================================
PODVector<LightProbe::GeomData> LightProbe::geomData_;
SharedArrayPtr<unsigned short> LightProbe::indexBuff_;
unsigned LightProbe::numIndeces_ = 0;
PODVector<LightProbe::SphericalData> LightProbe::sphericalData_;
Mutex LightProbe::sphDataLock_;

//=============================================================================
//=============================================================================
LightProbe::LightProbe(Context* context)
    : StaticModel(context)
    , generated_(false)
    , buildState_(SHBuild_Uninit)
    , dumpShCoeff_(false)
{
}

LightProbe::~LightProbe()
{
    threadProcess_ = NULL;
}

void LightProbe::RegisterObject(Context* context)
{
    context->RegisterFactory<LightProbe>();
}

void LightProbe::GenerateSH(const String &basepath, const String &fullpath)
{
    basepath_ = basepath;

    // 1st step in the process
    SetState(SHBuild_CubeCapture);
    cubeCapture_ = node_->GetOrCreateComponent<CubeCapture>();
    cubeCapture_->SetFilePath(ToString("node%u", node_->GetID()), basepath, fullpath);
    cubeCapture_->Start();

    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(LightProbe, HandleUpdate));
}

unsigned LightProbe::GetState()
{
    MutexLock lock(mutexStateLock_);
    return buildState_;
}

void LightProbe::SetState(unsigned state)
{
    MutexLock lock(mutexStateLock_);
    buildState_ = state;
}

void LightProbe::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    ForegroundProcess();
}

void LightProbe::ForegroundProcess()
{
    switch (GetState())
    {
    case SHBuild_CubeCapture:
        if (cubeCapture_ && cubeCapture_->IsFinished())
        {
            BeginSHBuildProcess();

            SetState(SHBuild_BackgroundProcess);
        }
        break;

    case SHBuild_FinalizeCoeff:
        {
            EndSHBuild();

            SetState(SHBuild_Complete);
        }
        break;
    }
}

void LightProbe::BackgroundProcess(void *data)
{
    LightProbe *parent = (LightProbe*)data;

    if (parent->GetState() == SHBuild_BackgroundProcess)
    {
        int nsamples = SetupSphericalData(parent->GetCubeImages(), parent->GetCoeffVec());

        if (nsamples == 0)
        {
            nsamples = CalculateSH(parent->GetCubeImages(), parent->GetCoeffVec());
        }

        parent->SetNumSamples(nsamples);
        parent->SetState(SHBuild_FinalizeCoeff);
    }
}

void LightProbe::BeginSHBuildProcess()
{
    CopyTextureCube();

    ClearCoeff();

    // start the background thread
    CreateThread();
}

void LightProbe::EndSHBuild()
{
    // done with the thread
    DestroyThread();

    FinalizeCoeff();

    UnsubscribeFromEvent(E_UPDATE);

    // send event
    using namespace SHBuildDone;

    VariantMap& eventData = GetEventDataMap();
    eventData[P_NODE] = node_;
    SendEvent(E_SHBUILDDONE, eventData);

    // dbg
    if (dumpShCoeff_)
    {
        DumpSHCoeff();
    }
}

void LightProbe::CopyTextureCube()
{
    // get the images from the texturecube
    cubeImages_.Resize(MAX_CUBEMAP_FACES);

    for ( unsigned i = 0; i < MAX_CUBEMAP_FACES; ++i )
    {
        cubeImages_[i] = cubeCapture_->GetTextureCube()->GetImage(CubeMapFace(i));
    }

    // done with cube capture
    node_->RemoveComponent(cubeCapture_);
    cubeCapture_ = NULL;
}

void LightProbe::CreateThread()
{
    threadProcess_ = new HelperThread<LightProbe>(this, &LightProbe::BackgroundProcess);
    threadProcess_->Start();
}

void LightProbe::DestroyThread()
{
    threadProcess_ = NULL;
}

void LightProbe::ClearCoeff()
{
    coeffVec_.Resize(9);
    for ( unsigned i = 0; i < 9; ++i )
    {
        coeffVec_[i] = Vector3::ZERO;
    }
}

void LightProbe::FinalizeCoeff()
{
    const float factor = 4.0f * M_PI /(float)(numSamples_);

    for ( int i = 0; i < 9; ++i )
    {
        coeffVec_[i] *= factor;
    }
}

void LightProbe::DumpSHCoeff()
{
    URHO3D_LOGINFOF("---------- node %u sh ----------", node_->GetID());
    char buff[50];
    for ( int i = 0; i < 9; ++i )
    {
        sprintf(buff, "%d: {%8.5f, %8.5f, %8.5f}", i, coeffVec_[i].x_, coeffVec_[i].y_, coeffVec_[i].z_);
        URHO3D_LOGINFO(String(buff));
    }
}

//=============================================================================
//=============================================================================
// static fns below this pt
//=============================================================================
void LightProbe::SetupUnitBoxGeom(Context *context)
{
    ResourceCache* cache = context->GetSubsystem<ResourceCache>();
    Model *model = cache->GetResource<Model>("Models/Box.mdl");
    Geometry *geometry = model->GetGeometry(0, 0);
    VertexBuffer *vbuffer = geometry->GetVertexBuffer(0);
    const unsigned char *vertexData = (const unsigned char*)vbuffer->Lock(0, vbuffer->GetVertexCount());

    // populate geom data
    if (vertexData)
    {
        unsigned elementMask = vbuffer->GetElementMask();
        unsigned numVertices = vbuffer->GetVertexCount();
        unsigned vertexSize = vbuffer->GetVertexSize();

        geomData_.Resize(numVertices);

        for ( unsigned i = 0; i < numVertices; ++i )
        {
            unsigned char *dataAlign = (unsigned char *)(vertexData + i * vertexSize);
            GeomData &geom = geomData_[i];

            if (elementMask & MASK_POSITION)
            {
                geom.pos_ = *reinterpret_cast<const Vector3*>(dataAlign);
                dataAlign += sizeof(Vector3);
            }
            if (elementMask & MASK_NORMAL)
            {
                geom.normal_ = *reinterpret_cast<const Vector3*>(dataAlign);
                dataAlign += sizeof(Vector3);
            }
            if (elementMask & MASK_COLOR)
            {
                dataAlign += sizeof(unsigned);
            }
            if (elementMask & MASK_TEXCOORD1)
            {
                geom.uv_ = *reinterpret_cast<const Vector2*>(dataAlign);
            }
        }
        vbuffer->Unlock();
    }

    // get indices
    IndexBuffer *ibuffer = geometry->GetIndexBuffer();
    const unsigned short *ushortData = (const unsigned short *)ibuffer->Lock(0, ibuffer->GetIndexCount());

    if (ushortData)
    {
        numIndeces_ = ibuffer->GetIndexCount();
        unsigned indexSize = ibuffer->GetIndexSize();
        indexBuff_ = new unsigned short[numIndeces_];
        memcpy(indexBuff_.Get(), ushortData, numIndeces_ * indexSize);

        ibuffer->Unlock();
    }
}

int LightProbe::CalculateSH(const Vector<SharedPtr<Image> > &cubeImages, PODVector<Vector3> &coeffVec)
{
    // build sh coeff
    for( unsigned i = 0; i < sphericalData_.Size(); ++i )
    {
        const SphericalData &sd = sphericalData_[i];

        UpdateCoeffs(cubeImages[sd.face_]->GetPixel(sd.x_, sd.y_).ToVector3(), sd.normal_, coeffVec);
    }

    return (int)sphericalData_.Size();
}

int LightProbe::SetupSphericalData(const Vector<SharedPtr<Image> > &cubeImages, PODVector<Vector3> &coeffVec)
{
    MutexLock lock(sphDataLock_);

    // 1st thread to encounter size == 0 will build it
    if (sphericalData_.Size() == 0)
    {
        const int texSizeX = cubeImages[0]->GetWidth();
        const int texSizeY = cubeImages[0]->GetHeight();
        const float texSizeXINV = 1.0f/(float)texSizeX;
        const float texSizeYINV = 1.0f/(float)texSizeY;

        // reserve 5% over what's expected
        sphericalData_.Reserve((int)((float)texSizeX *(float)texSizeY * 6.0f * 1.05f));

        // build sh coeff
        for( unsigned i = 0; i < numIndeces_; i += 3 )
        {
            const unsigned short idx0 = indexBuff_[i+0];
            const unsigned short idx1 = indexBuff_[i+1];
            const unsigned short idx2 = indexBuff_[i+2];

            const Vector3 &v0 = geomData_[idx0].pos_;	
            const Vector3 &v1 = geomData_[idx1].pos_;	
            const Vector3 &v2 = geomData_[idx2].pos_;	

            const Vector3 &n0 = geomData_[idx0].normal_;   
            //const Vector3 &n1 = geomData_[idx1].normal_;   
            //const Vector3 &n2 = geomData_[idx2].normal_;  

            const Vector2 &uv0 = geomData_[idx0].uv_;
            const Vector2 &uv1 = geomData_[idx1].uv_;
            const Vector2 &uv2 = geomData_[idx2].uv_;

            float xMin = 1.0f;	
            float xMax = 0.0f;	
            float yMin = 1.0f;
            float yMax = 0.0f;

            if (uv0.x_ < xMin) xMin = uv0.x_; 
            if (uv1.x_ < xMin) xMin = uv1.x_; 
            if (uv2.x_ < xMin) xMin = uv2.x_; 

            if (uv0.x_ > xMax) xMax = uv0.x_; 
            if (uv1.x_ > xMax) xMax = uv1.x_; 
            if (uv2.x_ > xMax) xMax = uv2.x_; 

            if (uv0.y_ < yMin) yMin = uv0.y_; 
            if (uv1.y_ < yMin) yMin = uv1.y_; 
            if (uv2.y_ < yMin) yMin = uv2.y_; 

            if (uv0.y_ > yMax) yMax = uv0.y_;
            if (uv1.y_ > yMax) yMax = uv1.y_;
            if (uv2.y_ > yMax) yMax = uv2.y_;

            const int pixMinX = (int)Max((float)floor(xMin*texSizeX)-1, 0.0f); 
            const int pixMaxX = (int)Min((float)ceil(xMax*texSizeX)+1, (float)texSizeX); 
            const int pixMinY = (int)Max((float)floor(yMin*texSizeY)-1, 0.0f); 
            const int pixMaxY = (int)Min((float)ceil(yMax*texSizeY)+1, (float)texSizeY);

            // get cur face image
            CubeMapFace face = GetCubefaceFromNormal(n0);
            const SharedPtr<Image> curfaceImage = cubeImages[face];
            Vector3 normal, bary;

            for ( int x = pixMinX; x < pixMaxX; ++x ) 
            {
                for ( int y = pixMinY; y < pixMaxY; ++y ) 
                {
                    bary = Barycentric(uv0, uv1, uv2, Vector2((float)x * texSizeXINV, (float)y * texSizeYINV));

                    if (BaryInsideTriangle(bary))
                    {
                        normal = (bary.x_ * v0 + bary.y_ * v1 + bary.z_ * v2).Normalized();
                        
                        UpdateCoeffs(curfaceImage->GetPixel(x, y).ToVector3(), normal, coeffVec);

                        // save sph data
                        sphericalData_.Resize(sphericalData_.Size() + 1);
                        SphericalData &sphData = sphericalData_[sphericalData_.Size() - 1];

                        sphData.x_      = x;
                        sphData.y_      = y;
                        sphData.face_   = face;
                        sphData.normal_ = normal;
                    }
                }
            }
        }

        return (int)sphericalData_.Size();
    }

    return 0;
}

//=============================================================================
// based on: An Efficient Representation for Irradiance Environment Maps
// http://graphics.stanford.edu/papers/envmap/
// 
// **note**
// we're not multiplying individual updates with domega. instead, it's multiplied at the final stage
//=============================================================================
void LightProbe::UpdateCoeffs(const Vector3 &vcol, const Vector3 &v, PODVector<Vector3> &coeffVec)
{
    const float c0  = 0.282095f;
    const float c1  = 0.488603f;
    const float c2  = 1.092548f;
    const float c3  = 0.315392f;
    const float c33 = c3 * 3.0f;
    const float c4  = 0.546274f;

    /* L_{00}.  Note that Y_{00} = 0.282095 */
    coeffVec[0] += vcol*c0;

    /* L_{1m}. -1 <= m <= 1.  The linear terms */
    coeffVec[1] += vcol*(c1*v.y_);   /* Y_{1-1} = 0.488603 y  */
    coeffVec[2] += vcol*(c1*v.z_);   /* Y_{10}  = 0.488603 z  */
    coeffVec[3] += vcol*(c1*v.x_);   /* Y_{11}  = 0.488603 x  */

    /* The Quadratic terms, L_{2m} -2 <= m <= 2 */

    /* First, L_{2-2}, L_{2-1}, L_{21} corresponding to xy,yz,xz */
    coeffVec[4] += vcol*(c2*v.x_*v.y_); /* Y_{2-2} = 1.092548 xy */ 
    coeffVec[5] += vcol*(c2*v.y_*v.z_); /* Y_{2-1} = 1.092548 yz */ 
    coeffVec[7] += vcol*(c2*v.x_*v.z_); /* Y_{21}  = 1.092548 xz */ 

    /* L_{20}.  Note that Y_{20} = 0.315392 (3z^2 - 1) */
    coeffVec[6] += vcol*(c33*v.z_*v.z_-c3); 

    /* L_{22}.  Note that Y_{22} = 0.546274 (x^2 - y^2) */
    coeffVec[8] += vcol*(c4*(v.x_*v.x_-v.y_*v.y_));
}

CubeMapFace LightProbe::GetCubefaceFromNormal(const Vector3 &normal)
{
    const Vector3 cubefaceNormals[MAX_CUBEMAP_FACES] = 
    {
        Vector3::RIGHT,
        Vector3::LEFT,
        Vector3::UP,
        Vector3::DOWN,
        Vector3::FORWARD,
        Vector3::BACK
    };

    for ( unsigned i = 0; i < MAX_CUBEMAP_FACES; ++i )
    {
        if (normal.DotProduct(cubefaceNormals[i]) > 1.0f - M_LARGE_EPSILON)
            return (CubeMapFace)i;
    }

    return FACE_POSITIVE_X;
}

