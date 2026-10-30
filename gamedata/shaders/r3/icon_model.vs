// Render to texture (hud_preview.capture): model vertex shader, all skinning modes.
#include "common.h"
#include "skin.h"

struct v2p
{
	float2 tc0  : TEXCOORD0;
	float3 n    : TEXCOORD1; // view space normal
	float4 hpos : SV_Position;
};

v2p _main(v_model v)
{
	v2p o;
	o.hpos = mul(m_WVP, v.P);
	o.tc0 = v.tc.xy;
	o.n = mul(m_WV, float4(v.N.xyz, 0));
	return o;
}

#ifdef SKIN_NONE
v2p main(v_model v) { return _main(v); }
#endif
#ifdef SKIN_0
v2p main(v_model_skinned_0 v) { return _main(skinning_0(v)); }
#endif
#ifdef SKIN_1
v2p main(v_model_skinned_1 v) { return _main(skinning_1(v)); }
#endif
#ifdef SKIN_2
v2p main(v_model_skinned_2 v) { return _main(skinning_2(v)); }
#endif
#ifdef SKIN_3
v2p main(v_model_skinned_3 v) { return _main(skinning_3(v)); }
#endif
#ifdef SKIN_4
v2p main(v_model_skinned_4 v) { return _main(skinning_4(v)); }
#endif
