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
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/RenderSurface.h>
#include <Urho3D/Graphics/Viewport.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Texture2D.h>
#include <Urho3D/Graphics/TextureCube.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/Resource/XMLFile.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/File.h>

#include "CubeCapture.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
#define FIXED_IMAGE_SIZE    32

//=============================================================================
// adapted from EditorCubeCapture.as
//=============================================================================
CubeCapture::CubeCapture(Context* context)
    : Component(context)
    , imgSize_(FIXED_IMAGE_SIZE)
    , updateCycle_(0)
    , finished_(false)
    , dumpOutputFiles_(false)
{
}

CubeCapture::~CubeCapture()
{
}

void CubeCapture::RegisterObject(Context* context)
{
    context->RegisterFactory<CubeCapture>();
}

void CubeCapture::SetFilePath(const String &filename, const String &basepath, const String &fullpath)
{
    filename_ = filename;
    basepath_ = basepath + "/Cubemaps";
    fullpath_ = fullpath;
}

String CubeCapture::GetTextureCubeName()
{
    String xmlPath = basepath_ + "/" + filename_ + ".xml";
    return xmlPath;
}

void CubeCapture::Start()
{
    camNode_ = GetScene()->CreateChild("RenderCamera");
    camera_ = camNode_->GetOrCreateComponent<Camera>();
    camera_->SetFov(90.0f);
    camera_->SetNearClip(0.0001f);
    camera_->SetAspectRatio(1.0f);
    camNode_->SetWorldPosition(node_->GetWorldPosition());

    viewport_ = new Viewport(context_, GetScene(), camera_);
    viewport_->SetRenderPath(GetSubsystem<Renderer>()->GetViewport(0)->GetRenderPath());

    // Construct render surface 
    renderImage_ = new Texture2D(context_);
    renderImage_->SetSize(imgSize_, imgSize_, Graphics::GetRGBAFormat(), TEXTURE_RENDERTARGET);
    
    renderSurface_ = renderImage_->GetRenderSurface();
    renderSurface_->SetViewport(0, viewport_);
    renderSurface_->SetUpdateMode(SURFACE_UPDATEALWAYS);

    // textureCube
    textureCube_ = new TextureCube(context_);

    SubscribeToEvent(E_BEGINFRAME, URHO3D_HANDLER(CubeCapture, HandlePreRender));
    SubscribeToEvent(E_ENDFRAME, URHO3D_HANDLER(CubeCapture, HandlePostRender));
}

void CubeCapture::Stop()
{
    camNode_->Remove();
    camNode_ = NULL;
    viewport_ = NULL;
    renderSurface_ = NULL;
    finished_ = true;
    
    // generate output file
    if (dumpOutputFiles_)
    {
        WriteXML();
    }

    UnsubscribeFromEvent(E_BEGINFRAME);
    UnsubscribeFromEvent(E_ENDFRAME);
}

void CubeCapture::HandlePreRender(StringHash eventType, VariantMap& eventData)
{
    if (camNode_)
    {
        if (updateCycle_ < MAX_CUBEMAP_FACES)
        {
            camNode_->SetWorldRotation(RotationOf(CubeMapFace(updateCycle_)));
        }
        else
        {
            Stop();
        }
    }
}

void CubeCapture::HandlePostRender(StringHash eventType, VariantMap& eventData)
{
    CubeMapFace face = CubeMapFace(updateCycle_);
    textureCube_->SetData(face, renderImage_->GetImage(), false);

    // generate output file
    if (dumpOutputFiles_)
    {
        String path = fullpath_ + "/" + basepath_ + "/" + filename_ + "_" + GetFaceName(face) + ".png";
        textureCube_->GetImage(face)->SavePNG(path);
    }

    // post increment, opposed to how it's pre-incremented in the EditorCubeCapture.as
    ++updateCycle_;
}

void CubeCapture::WriteXML()
{
    String cubeName;
    String xmlPath = fullpath_ + "/" + basepath_ + "/"  + filename_ + ".xml";
    String simpleBasepath = basepath_;
    if (simpleBasepath.StartsWith("Data/"))
    {
        simpleBasepath = String(simpleBasepath).Substring(5);
    }

    SharedPtr<XMLFile> file(new XMLFile(context_));
    XMLElement rootElem = file->CreateRoot("cubemap");
    
    for ( int i = 0; i < MAX_CUBEMAP_FACES; ++i )
    {
        XMLElement faceElem = rootElem.CreateChild("face");
        faceElem.SetAttribute("name", simpleBasepath + "/" + filename_ + "_" + GetFaceName((CubeMapFace)i) + ".png");
    }

    SharedPtr<File> outfile(new File(context_, xmlPath, FILE_WRITE));
    file->Save(*outfile, "    ");
}

String CubeCapture::GetFaceName(CubeMapFace face) const
{
    switch (face)
    {
    case FACE_POSITIVE_X: return "PosX";
    case FACE_POSITIVE_Y: return "PosY";                                   
    case FACE_POSITIVE_Z: return "PosZ";
    case FACE_NEGATIVE_X: return "NegX";
    case FACE_NEGATIVE_Y: return "NegY";
    case FACE_NEGATIVE_Z: return "NegZ";
    }
    return "PosX";
}

Quaternion CubeCapture::RotationOf(CubeMapFace face) const
{
    switch (face)
    {
    //  Rotate camera according to probe rotation
    case FACE_POSITIVE_X: return Quaternion(0, 90, 0); 
    case FACE_NEGATIVE_X: return Quaternion(0, -90, 0);
    case FACE_POSITIVE_Y: return Quaternion(-90, 0, 0);
    case FACE_NEGATIVE_Y: return Quaternion(90, 0, 0); 
    case FACE_POSITIVE_Z: return Quaternion(0, 0, 0);  
    case FACE_NEGATIVE_Z: return Quaternion(0, 180, 0);
    }
    return Quaternion::IDENTITY;
}

