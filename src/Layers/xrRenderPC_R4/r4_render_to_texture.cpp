#include "stdafx.h"
#include "../xrRender/FBasicVisual.h"
#include "../xrRender/FHierrarhyVisual.h"
#include "../../xrEngine/Fmesh.h"

// Render to texture (IRender_interface::RenderToTexture): visuals into a named render target, outside the frame.
// Own forward shaders icon_model / icon_model_aref (shaders\r3): base texture and a simple studio light, so the
// result does not depend on the scene, the deferred targets or MSAA. Called from scripts (hud_preview.capture),
// on the main thread between frames; the backend state it touches is saved and restored.

namespace
{
class CBlender_render_to_texture : public IBlender
{
public:
	explicit CBlender_render_to_texture(LPCSTR shader) : shader(shader) { description.CLS = 0; }
	LPCSTR getComment() override { return "render to texture: model"; }
	BOOL canBeDetailed() override { return FALSE; }
	BOOL canBeLMAPped() override { return FALSE; }
	void Compile(CBlender_Compile& C) override
	{
		IBlender::Compile(C);
		C.r_Pass("icon_model", shader, FALSE, TRUE, TRUE); // one vertex shader, pixel shader by alpha test
		C.r_dx10Texture("s_base", C.L_textures[0]);
		C.r_dx10Sampler("smp_base");
		C.r_End();
	}
private:
	LPCSTR shader;
};

CBlender_render_to_texture blender_opaque("icon_model");
CBlender_render_to_texture blender_aref("icon_model_aref");

struct RttTarget
{
	ref_rt color, depth;
	ID3D11Texture2D* staging = nullptr;
	u32 width = 0, height = 0;

	void release()
	{
		_RELEASE(staging);
		color = nullptr;
		depth = nullptr;
		width = height = 0;
	}
};

xr_map<shared_str, RttTarget> targets;
xr_map<shared_str, ref_shader> shaders; // by base texture, alpha test and skinning

bool alpha_tested(const dxRender_Visual* V)
{
	LPCSTR s = V->dbg_shader.c_str();
	return s && (strstr(s, "aref") || strstr(s, "transparent") || strstr(s, "foliage"));
}

ref_shader* shader_for(dxRender_Visual* V)
{
	LPCSTR tex = V->dbg_texture.c_str();
	if (!tex || !tex[0])
		return nullptr;
	const bool aref = alpha_tested(V);
	string_path key;
	xr_sprintf(key, "%s|%d|%d", tex, int(aref), V->skinning);
	ref_shader& s = shaders[shared_str(key)];
	if (!s)
	{
		// vertex shaders are compiled for the skinning mode current at creation (SKIN_x), as the model's own
		const int previous = Engine.External.GetSkinningMode();
		Engine.External.SetSkinningMode(V->skinning);
		s.create(aref ? &blender_aref : &blender_opaque, aref ? "icon_model_aref" : "icon_model", tex);
		Engine.External.SetSkinningMode(previous);
	}
	return s ? &s : nullptr;
}

void draw(dxRender_Visual* V, const Fmatrix& world, const Fvector4& light)
{
	if (!V)
		return;
	switch (V->Type)
	{
	case MT_HIERRARHY:
	case MT_SKELETON_ANIM:
	case MT_SKELETON_RIGID:
		// children only: hidden parts (all bones of a child hidden) are in children_invisible
		for (IRenderVisual* c : static_cast<FHierrarhyVisual*>(V)->children)
			draw(static_cast<dxRender_Visual*>(c), world, light);
		break;
	case MT_LOD:
	case MT_PARTICLE_EFFECT:
	case MT_PARTICLE_GROUP:
	case MT_TREE_ST:
	case MT_TREE_PM:
	case MT_3DFLUIDVOLUME:
		break;
	default:
		if (ref_shader* s = shader_for(V))
		{
			RCache.set_Shader(*s);
			RCache.set_xform_world(world);
			RCache.set_c("icon_light", light);
			V->Render(1.f);
		}
		break;
	}
}

bool prepare(RttTarget& t, LPCSTR name, u32 w, u32 h)
{
	if (t.color && t.width == w && t.height == h)
		return true;
	t.release();
	string_path depth_name;
	xr_sprintf(depth_name, "%s_depth", name);
	t.color.create(name, w, h, D3DFMT_A8R8G8B8);
	t.depth.create(depth_name, w, h, D3DFMT_D24S8);
	if (!t.color || !t.color->pSurface || !t.color->pRT || !t.depth || !t.depth->pZRT)
	{
		t.release();
		return false;
	}
	D3D11_TEXTURE2D_DESC desc;
	t.color->pSurface->GetDesc(&desc);
	desc.Usage = D3D11_USAGE_STAGING;
	desc.BindFlags = 0;
	desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
	desc.MiscFlags = 0;
	if (FAILED(HW.pDevice->CreateTexture2D(&desc, nullptr, &t.staging)))
		t.staging = nullptr;
	t.width = w;
	t.height = h;
	return true;
}

// pixel bounds of alpha > 0
void opaque_bounds(RttTarget& t, Irect& r)
{
	r.set(0, 0, -1, -1);
	if (!t.staging)
		return;
	HW.pContext->CopyResource(t.staging, t.color->pSurface);
	D3D11_MAPPED_SUBRESOURCE m;
	if (FAILED(HW.pContext->Map(t.staging, 0, D3D11_MAP_READ, 0, &m)))
		return;
	int x1 = int(t.width), y1 = int(t.height), x2 = -1, y2 = -1;
	for (u32 y = 0; y < t.height; ++y)
	{
		const u8* row = static_cast<const u8*>(m.pData) + y * m.RowPitch;
		for (u32 x = 0; x < t.width; ++x)
			if (row[x * 4 + 3])
			{
				if (int(x) < x1) x1 = int(x);
				if (int(x) > x2) x2 = int(x);
				if (int(y) < y1) y1 = int(y);
				y2 = int(y);
			}
	}
	HW.pContext->Unmap(t.staging, 0);
	if (x2 >= x1)
		r.set(x1, y1, x2, y2);
}
}

bool CRender::RenderToTexture(LPCSTR texture, u32 width, u32 height, const Fmatrix& view, const Fmatrix& project,
	const RenderToTextureItem* items, u32 count, const Fvector4& light, Irect& opaque)
{
	opaque.set(0, 0, -1, -1);
	if (!Device.b_is_Ready || !texture || !texture[0] || !width || !height || width > 4096 || height > 4096)
		return false;
	RttTarget& t = targets[shared_str(texture)];
	if (!prepare(t, texture, width, height))
		return false;

	// save
	ID3DRenderTargetView* outputs[4];
	for (u32 i = 0; i < 4; ++i)
		outputs[i] = RCache.get_RT(i);
	ID3DDepthStencilView* depth = RCache.get_ZB();
	UINT viewports = 1;
	D3D11_VIEWPORT viewport{};
	HW.pContext->RSGetViewports(&viewports, &viewport);
	const Fmatrix world0 = RCache.xforms.m_w, view0 = RCache.xforms.m_v, project0 = RCache.xforms.m_p;

	// draw
	RCache.set_RT(t.color->pRT, 0);
	for (u32 i = 1; i < 4; ++i)
		RCache.set_RT(nullptr, i);
	RCache.set_ZB(t.depth->pZRT);
	const D3D11_VIEWPORT vp{ 0.f, 0.f, float(width), float(height), 0.f, 1.f };
	HW.pContext->RSSetViewports(1, &vp);
	const FLOAT clear[4] = { 0.f, 0.f, 0.f, 0.f };
	HW.pContext->ClearRenderTargetView(t.color->pRT, clear);
	HW.pContext->ClearDepthStencilView(t.depth->pZRT, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.f, 0);
	RCache.set_xform_view(view);
	RCache.set_xform_project(project);
	for (u32 i = 0; i < count; ++i)
		draw(static_cast<dxRender_Visual*>(items[i].visual), items[i].world, light);

	// restore
	RCache.Invalidate();
	for (u32 i = 0; i < 4; ++i)
		RCache.set_RT(outputs[i], i);
	RCache.set_ZB(depth);
	if (viewports)
		HW.pContext->RSSetViewports(1, &viewport);
	RCache.set_xform_world(world0);
	RCache.set_xform_view(view0);
	RCache.set_xform_project(project0);

	opaque_bounds(t, opaque);
	return true;
}

void RenderToTextureShutdown() // CRender::destroy: before the resource manager goes
{
	for (auto& it : targets)
		it.second.release();
	targets.clear();
	shaders.clear();
}

void CRender::ReleaseRenderTexture(LPCSTR texture)
{
	auto it = texture ? targets.find(shared_str(texture)) : targets.end();
	if (it == targets.end())
		return;
	it->second.release();
	targets.erase(it);
}
