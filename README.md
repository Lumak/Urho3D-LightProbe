# Urho3D LightProbe
  
---
### Description
SH coefficients generated from cubemap texture to achieve irradiance used as light probe. Based on: An Efficient Representation for Irradiance Environment Maps,  
ref: http://graphics.stanford.edu/papers/envmap/  

There are six lightprobes in the scene that reflect some color to validate testing. You can spot them easily.  
  
---  
### How the coffecients are generated, stored and applied:
1) CubeCapture class generates cubemap textures.
2) LightProbe class maps the texture onto a unit box and generates SH coefficients onto a spherical space.
3) LightProbeCreator class gathers SH coefficients from all the LightProbes and packs the data into a single ShprobeData.png file.
4) shader program reads the ShprobeData.png data and applies irradiance (eqn. 13) mentioned in the above ref.
5) Character class periodically searches for the nearest light probe and updates shader params.
  
Coefficient generation takes about **~170 msec.** to generate six light probe coeffs in the scene. Your results may vary. The example does not generate the coefficients automatically, as it's already generated.  
To enable coeff generation, set **generateLightProbes_=true** in the CharacterDemo class.  

#### Some useful debugging info:
* dump cubemap textures by setting **dumpOutputFiles_=true** in CubeCapture class.
* dump sh coeffs by setting **dumpShCoeff_=true** in LightProbe class.  
**Note:** enabling the above dump will obviously impact the build time.  
  
---  
### DX9 build problems:
* huge shader compile spike when you 1st run the demo. Created MANUAL_UNROLL preprocessor define and it's set to on, currently.
* light globes, models which cover actual lights, changes to different color. I have no idea why it does this.
* initially declared cProbeIndex as int and I noticed it was glitchy. Changed it to float and it works fine. Consequently, changed glsl var to float to reflect shader programs to remain consistent.

  
---
### Screenshot

![alt tag](https://github.com/Lumak/Urho3D-LightProbe/blob/master/screenshot/lightprobescreen0.png)

---
### To Build
To build it, unzip/drop the repository into your Urho3D/ folder and build it the same way as you'd build the default Samples that come with Urho3D.
**Built with Urho3D 1.7 tag.**
  
---  
### License
The MIT License (MIT)







