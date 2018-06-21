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
#include <Urho3D/Scene/Component.h>
#include <Urho3D/Graphics/TextureCube.h>

namespace Urho3D
{
class Scene;
class Camera;
class Viewport;
class Texture2D;
class RenderSurface;
}

using namespace Urho3D;

//=============================================================================
//=============================================================================
class CubeCapture : public Component
{
    URHO3D_OBJECT(CubeCapture, Component);

public:
    CubeCapture(Context* context);
    virtual ~CubeCapture();

    static void RegisterObject(Context* context);

    void SetFilePath(const String &filename, const String &basepath, const String &fullpath);
    void Start();
    bool IsFinished() const                         { return finished_; }

    SharedPtr<TextureCube> GetTextureCube() const   { return textureCube_; }
    String GetTextureCubeName();

    void SetDumpOutputFiles(bool dump)              { dumpOutputFiles_ = dump; }
    bool GetDumpOutputFiles() const                 { return dumpOutputFiles_; }

protected:
    void Stop();
    void HandlePreRender(StringHash eventType, VariantMap& eventData);
    void HandlePostRender(StringHash eventType, VariantMap& eventData);
    void WriteXML();
    String GetFaceName(CubeMapFace face) const;
    Quaternion RotationOf(CubeMapFace face) const;

protected:
    String                  filename_;
    String                  fullpath_;
    String                  basepath_;

    WeakPtr<Node>           camNode_;
    SharedPtr<Camera>       camera_;
    SharedPtr<Viewport>     viewport_;
    SharedPtr<Texture2D>    renderImage_;
    WeakPtr<RenderSurface>  renderSurface_;
    SharedPtr<TextureCube>  textureCube_;

    int                     updateCycle_;
    int                     imgSize_;
    String                  imagePath_;
    bool                    finished_;

    // dbg
    bool                    dumpOutputFiles_;
};
