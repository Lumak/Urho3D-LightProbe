#ifdef COMPILEPS
//=============================================================================
//=============================================================================
uniform float cProbeIndex;
uniform vec3 cProbePosition;
uniform float cMinProbeDistance;
uniform float cSHIntensity;
uniform float cTextureSize;

#line 1000
//=============================================================================
// Based on: An Efficient Representation for Irradiance Environment Maps.  
// ref: http://graphics.stanford.edu/papers/envmap/
//=============================================================================
vec3 IrradCoeffs(vec3 L00, vec3 L1_1, vec3 L10, vec3 L11, 
                 vec3 L2_2, vec3 L2_1, vec3 L20, vec3 L21, vec3 L22,
                 vec3 n) 
{
  //------------------------------------------------------------------
  // These are variables to hold x,y,z and squares and products

	float x2 ;
	float  y2 ;
	float z2 ;
	float xy ;
	float  yz ;
	float  xz ;
	float x ;
	float y ;
	float z ;
	vec3 col ;
  //------------------------------------------------------------------       
  // We now define the constants and assign values to x,y, and z 
	
	const float c1 = 0.429043 ;
	const float c2 = 0.511664 ;
	const float c3 = 0.743125 ;
	const float c4 = 0.886227 ;
	const float c5 = 0.247708 ;
	x = n.x ; y = n.y ; z = n.z ;
  //------------------------------------------------------------------ 
  // We now compute the squares and products needed 

	x2 = x*x ; y2 = y*y ; z2 = z*z ;
	xy = x*y ; yz = y*z ; xz = x*z ;
  //------------------------------------------------------------------ 
  // Finally, we compute equation 13

	col = c1*L22*(x2-y2) + c3*L20*z2 + c4*L00 - c5*L20 
            + 2*c1*(L2_2*xy + L21*xz + L2_1*yz) 
            + 2*c2*(L11*x+L1_1*y+L10*z) ;

	return col;
}

vec3 GetSH(int i)
{
    #ifdef GL_ES
    vec3 sh = texture2D(sEnvMap, vec2(float(cProbeIndex*9 + i), 0)/cTextureSize).xyz;
    #else
    vec3 sh = texelFetch(sEnvMap, ivec2(cProbeIndex*9 + i, 0), 0).xyz;
    #endif
    sh = (sh - vec3(0.5, 0.5, 0.5)) * 10.0f;
    return sh;
}

#line 2000
vec3 SHDiffuse(vec3 normal, vec3 worldPos)
{
    // world pos
    float dist = max(0.75, length(cProbePosition - worldPos));
    const float falloffDist = 1.5;

    if (dist > cMinProbeDistance + falloffDist)
    {
        return vec3(0,0,0);
    }

    // exponential falloff
    if (dist > cMinProbeDistance)
    {
        dist = cMinProbeDistance + pow(0.5 + (dist - cMinProbeDistance), 4);
    }

    // read sh
    vec3 sh[9];
    for (int i = 0; i < 9; ++i)
    {
        sh[i] = GetSH(i);
    }

    // linear decay 
    return IrradCoeffs(sh[0], sh[1], sh[2], sh[3], sh[4], sh[5], sh[6], sh[7], sh[8], normal) * cSHIntensity/dist;
}

#endif //COMPILEPS

