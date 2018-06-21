//
// Copyright (c) 2008-2016 the Urho3D project.
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

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Graphics/Technique.h>
#include <Urho3D/Graphics/Texture2D.h>
#include <Urho3D/Graphics/Terrain.h>
#include <Urho3D/Input/Controls.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/IO/MemoryBuffer.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/Engine/DebugHud.h>

#include "CharacterDemo.h"
#include "Character.h"
#include "LightProbeCreator.h"
#include "CollisionLayer.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
const float CAMERA_MIN_DIST = 1.0f;
const float CAMERA_INITIAL_DIST = 5.0f;
const float CAMERA_MAX_DIST = 20.0f;

//=============================================================================
//=============================================================================
URHO3D_DEFINE_APPLICATION_MAIN(CharacterDemo)

//=============================================================================
//=============================================================================
CharacterDemo::CharacterDemo(Context* context)
    : Sample(context)
    , firstPerson_(false)
    , drawDebug_(false)
    , cameraMode_(false)
    , generateLightProbes_(false)
{
    Character::RegisterObject(context);
}

CharacterDemo::~CharacterDemo()
{
}

void CharacterDemo::Setup()
{
    engineParameters_["WindowTitle"]   = GetTypeName();
    engineParameters_["LogName"]       = GetSubsystem<FileSystem>()->GetProgramDir() + "lightProbe.log";
    engineParameters_["FullScreen"]    = false;
    engineParameters_["Headless"]      = false;
    engineParameters_["WindowWidth"]   = 1280; 
    engineParameters_["WindowHeight"]  = 720;
    engineParameters_["ResourcePaths"] = "Data;CoreData;Data/LightProbe;";
}

void CharacterDemo::Start()
{
    Sample::Start();

    // init lp creator - this needs to be created before a scene is parsed, otherwise, LightProbe component is unknown
    CreateLightProbeCreator();

    CreateInstructions();

    CreateScene();

    if (!generateLightProbes_)
    {
        CreateCharacter();
    }

    SubscribeToEvents();

    Sample::InitMouseMode(MM_RELATIVE);

    ChangeDebugHudText();
}

void CharacterDemo::CreateScene()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    scene_ = new Scene(context_);

    cameraNode_ = new Node(context_);
    Camera* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(300.0f);
    GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, camera));

    // load scene
    XMLFile *xmlLevel = cache->GetResource<XMLFile>("LightProbe/testScene.xml");
    scene_->LoadXML(xmlLevel->GetRoot());

    //generateLightProbes_ = true;
    if (generateLightProbes_)
    {
        if (instructionText_)
        {
            instructionText_->SetText("building");
        }

        // init lp creator
        LightProbeCreator *lightProbeCreator = GetSubsystem<LightProbeCreator>();
        lightProbeCreator->Init(scene_, "Data/LightProbe");
        lightProbeCreator->SetOutputFilename(GetSubsystem<FileSystem>()->GetProgramDir() + "Data/LightProbe/Textures/SHprobeData.png");

        // start the timer and go
        hrTimer_.Reset();
        lightProbeCreator->GenerateLightProbes();
    }
}

void CharacterDemo::CreateCharacter()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    Node *spawnNode = scene_->GetChild("playerSpawn");
    Node* objectNode = scene_->CreateChild("Player");
    objectNode->SetPosition(spawnNode->GetPosition());

    // spin node
    Node* adjustNode = objectNode->CreateChild("spinNode");
    adjustNode->SetRotation( Quaternion(180, Vector3(0,1,0) ) );
    
    // Create the rendering component + animation controller
    AnimatedModel* object = adjustNode->CreateComponent<AnimatedModel>();
    object->SetModel(cache->GetResource<Model>("Platforms/Models/BetaLowpoly/Beta.mdl"));
    SharedPtr<Material> c1Mat = cache->GetResource<Material>("LightProbe/Materials/BetaBody_MAT.xml")->Clone();
    SharedPtr<Material> c2Mat = cache->GetResource<Material>("LightProbe/Materials/BetaJoints_MAT.xml")->Clone();
    object->SetMaterial(0, c1Mat);
    object->SetMaterial(1, c1Mat);
    object->SetMaterial(2, c2Mat);

    // set shader texture width param
    Texture* texture = c1Mat->GetTexture(TU_ENVIRONMENT);
    if (texture)
    {
        c1Mat->SetShaderParameter("TextureSize", (float)texture->GetWidth());
        c2Mat->SetShaderParameter("TextureSize", (float)texture->GetWidth());
    }

    object->SetCastShadows(true);
    adjustNode->CreateComponent<AnimationController>();

    // Create rigidbody, and set non-zero mass so that the body becomes dynamic
    RigidBody* body = objectNode->CreateComponent<RigidBody>();
    body->SetCollisionLayer(ColLayer_Character);
    body->SetCollisionMask(ColMask_Character);
    body->SetMass(1.0f);
    body->SetRollingFriction(1.0f);

    body->SetAngularFactor(Vector3::ZERO);
    body->SetCollisionEventMode(COLLISION_ALWAYS);

    // Set a capsule shape for collision
    CollisionShape* shape = objectNode->CreateComponent<CollisionShape>();
    shape->SetCapsule(0.7f, 1.8f, Vector3(0.0f, 0.90f, 0.0f));

    // character
    character_ = objectNode->CreateComponent<Character>();
    Vector3 euAngle = spawnNode->GetRotation().EulerAngles();
    character_->controls_.yaw_ = euAngle.y_;
}

void CharacterDemo::CreateInstructions()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    Graphics *graphics = GetSubsystem<Graphics>();
    UI* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    instructionText_ = ui->GetRoot()->CreateChild<Text>();
    instructionText_->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 12);
    instructionText_->SetTextAlignment(HA_CENTER);
    instructionText_->SetColor(Color::CYAN);

    // Position the text relative to the screen center
    instructionText_->SetHorizontalAlignment(HA_CENTER);
    instructionText_->SetPosition(0, 10);
}

void CharacterDemo::CreateLightProbeCreator()
{
    context_->RegisterSubsystem(new LightProbeCreator(context_));
}

void CharacterDemo::ChangeDebugHudText()
{
    // change profiler text
    if (GetSubsystem<DebugHud>())
    {
        Text *dbgText = GetSubsystem<DebugHud>()->GetProfilerText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetStatsText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetMemoryText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetModeText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);
    }
}

void CharacterDemo::SubscribeToEvents()
{
    if (generateLightProbes_)
    {
        SubscribeToEvent(E_LIGHTPROBESTATUS, URHO3D_HANDLER(CharacterDemo, HandleLPStatusEvent));
    }
    else
    {
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(CharacterDemo, HandleUpdate));
        SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(CharacterDemo, HandlePostUpdate));
        UnsubscribeFromEvent(E_SCENEUPDATE);
    }
}

void CharacterDemo::HandleLPStatusEvent(StringHash eventType, VariantMap& eventData)
{
    using namespace LightProbeStatus;
    unsigned totalCnt = eventData[P_TOTAL].GetUInt();
    unsigned completeCnt = eventData[P_COMPLETED].GetUInt();

    if (totalCnt == completeCnt)
    {
        float elapsed = (float)((long)hrTimer_.GetUSec(false))/1000.0f;
        char buff[30];
        sprintf(buff, "%.2f", elapsed);
        instructionText_->SetText("light probes build: " + String(buff) + " msec.");

        // init remainding
        CreateCharacter();

        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(CharacterDemo, HandleUpdate));
        SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(CharacterDemo, HandlePostUpdate));
        UnsubscribeFromEvent(E_SCENEUPDATE);
    }
    else
    {
        instructionText_->SetText(ToString("light probes complete: %u/%u", completeCnt, totalCnt));
    }
}

void CharacterDemo::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    Input* input = GetSubsystem<Input>();

    if (character_)
    {
        // Clear previous controls
        character_->controls_.Set(CTRL_FORWARD | CTRL_BACK | CTRL_LEFT | CTRL_RIGHT | CTRL_JUMP, false);

        // Update controls using keys
        UI* ui = GetSubsystem<UI>();
        if (!ui->GetFocusElement())
        {
            character_->controls_.Set(CTRL_FORWARD, input->GetKeyDown(KEY_W));
            character_->controls_.Set(CTRL_BACK, input->GetKeyDown(KEY_S));
            character_->controls_.Set(CTRL_LEFT, input->GetKeyDown(KEY_A));
            character_->controls_.Set(CTRL_RIGHT, input->GetKeyDown(KEY_D));
            character_->controls_.Set(CTRL_JUMP, input->GetKeyDown(KEY_SPACE));

            character_->controls_.yaw_ += (float)input->GetMouseMoveX() * YAW_SENSITIVITY;
            character_->controls_.pitch_ += (float)input->GetMouseMoveY() * YAW_SENSITIVITY;

            // Limit pitch
            character_->controls_.pitch_ = Clamp(character_->controls_.pitch_, -80.0f, 80.0f);
            // Set rotation already here so that it's updated every rendering frame instead of every physics frame
            character_->GetNode()->SetRotation(Quaternion(character_->controls_.yaw_, Vector3::UP));
        }
    }

    // Toggle debug geometry with space
    if (input->GetKeyPress(KEY_F5))
    {
        drawDebug_ = !drawDebug_;
    }
}

void CharacterDemo::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
    if (!character_)
        return;

    Node* characterNode = character_->GetNode();
    Quaternion rot = characterNode->GetRotation();
    Quaternion dir = rot * Quaternion(character_->controls_.pitch_, Vector3::RIGHT);

    {
        Vector3 aimPoint = characterNode->GetPosition() + rot * Vector3(0.0f, 1.7f, 0.0f);
        Vector3 rayDir = dir * Vector3::BACK;
        float rayDistance = CAMERA_INITIAL_DIST;
        PhysicsRaycastResult result;

        scene_->GetComponent<PhysicsWorld>()->RaycastSingle(result, Ray(aimPoint, rayDir), rayDistance, ColMask_Camera);
        if (result.body_)
            rayDistance = Min(rayDistance, result.distance_);
        rayDistance = Clamp(rayDistance, CAMERA_MIN_DIST, CAMERA_MAX_DIST);

        cameraNode_->SetPosition(aimPoint + rayDir * rayDistance);
        cameraNode_->SetRotation(dir);
    }
}

void CharacterDemo::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;

    Input* input = GetSubsystem<Input>();

    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    // Use the Translate() function (default local space) to move relative to the node's orientation.
    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);
}

