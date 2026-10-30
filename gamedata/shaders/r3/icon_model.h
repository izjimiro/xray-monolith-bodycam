// Render to texture (hud_preview.capture): studio light for icon_model*.ps
#include "common.h"

uniform float4 icon_light; // xyz - direction to the key light (view space), w - ambient

struct icon_v2p
{
	float2 tc0 : TEXCOORD0;
	float3 n   : TEXCOORD1;
};

float4 icon_shade(icon_v2p I, float4 t_base)
{
	float3 n = normalize(I.n);
	float3 key = normalize(icon_light.xyz);
	float3 fill = normalize(float3(-key.x, 0.2, -0.8));
	float3 rim = normalize(float3(0, 0.6, 0.8));
	float light = icon_light.w
		+ saturate(dot(n, key)) * 0.85
		+ saturate(dot(n, fill)) * 0.3
		+ saturate(dot(n, rim)) * 0.25;
	return float4(t_base.rgb * light, 1);
}
