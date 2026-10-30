#include "stdafx.h"
#include "hud_preview.h"
#include "player_hud.h"
#include "HudItem.h"
#include "script_game_object.h"
#include "../xrEngine/Render.h"
#include "../Include/xrRender/RenderVisual.h"
#include "Level.h"

namespace
{
	struct Node
	{
		IRenderVisual* visual = nullptr;
		shared_str parent;
		Fmatrix local = Fidentity;  // root: relative to the camera; child: offset from the parent
		Fmatrix world = Fidentity;
		u32 frame = u32(-1);
		u16 pose_src = u16(-1); // FollowPose: object id
	};

	xrCriticalSection lock; // Lua (main thread) vs HUD render
	xr_map<shared_str, Node*> nodes;
	Fmatrix camera = Fidentity;
	bool hide_hands = false;

	Node* find(LPCSTR id)
	{
		auto it = nodes.find(id);
		return it != nodes.end() ? it->second : nullptr;
	}

	IKinematics* kinematics(Node* n) { return n && n->visual ? n->visual->dcast_PKinematics() : nullptr; }

	// HUD model of a hand item if it has one, else the world model
	IKinematics* object_kinematics(CObject* o)
	{
		CHudItem* hi = o ? smart_cast<CHudItem*>(o) : nullptr;
		if (hi && hi->HudItemData())
			return hi->HudItemData()->m_model;
		return o && o->Visual() ? o->Visual()->dcast_PKinematics() : nullptr;
	}

	void copy_pose(IKinematics* dst, IKinematics* src)
	{
		for (u16 b = 0; b < dst->LL_BoneCount(); ++b)
		{
			const u16 s = src->LL_BoneID(dst->LL_BoneName_dbg(b));
			if (s == BI_NONE)
				continue;
			CBoneInstance& d = dst->LL_GetBoneInstance(b);
			const CBoneInstance& from = src->LL_GetBoneInstance(s);
			d.mTransform = from.mTransform;
			d.mRenderTransform = from.mRenderTransform;
		}
	}

	void destroy(Node* n)
	{
		if (n->visual)
			::Render->model_Delete(n->visual);
		xr_delete(n);
	}

	// rotation X, then Y, then Z, then translation (as attach_mount_hud_offset)
	Fmatrix xyz(const Fvector& pos, const Fvector& rot)
	{
		Fmatrix m, r;
		m.rotateX(rot.x);
		m.mulA_43(r.rotateY(rot.y));
		m.mulA_43(r.rotateZ(rot.z));
		m.translate_over(pos);
		return m;
	}

	const Fmatrix& resolve(Node* n, int depth = 0)
	{
		if (n->frame == Device.dwFrame)
			return n->world;
		n->frame = Device.dwFrame;
		Node* p = n->parent.size() && depth < 16 ? find(n->parent.c_str()) : nullptr;
		if (p)
			n->world.mul_43(resolve(p, depth + 1), n->local);
		else
			n->world.mul_43(camera, n->local);
		return n->world;
	}
}

bool HudPreview::Add(LPCSTR id, LPCSTR visual)
{
	IRenderVisual* v = nullptr;
	if (visual && visual[0])
	{
		string_path path;
		strconcat(sizeof(path), path, visual, strstr(visual, ".ogf") ? "" : ".ogf");
		if (!FS.exist("$game_meshes$", path))
		{
			Msg("! hud_preview.add [%s]: no visual [%s]", id, visual);
			return false;
		}
		::Render->hud_loading = true;
		v = ::Render->model_Create(visual);
		::Render->hud_loading = false;
		if (!v)
			return false;
	}
	xrCriticalSectionGuard g(lock);
	Node*& n = nodes[id];
	if (n)
	{
		if (n->visual)
			::Render->model_Delete(n->visual);
	}
	else
		n = xr_new<Node>();
	n->visual = v;
	n->frame = u32(-1);
	return true;
}

void HudPreview::Remove(LPCSTR id)
{
	xrCriticalSectionGuard g(lock);
	xr_vector<shared_str> doomed;
	doomed.push_back(shared_str(id));
	for (u32 i = 0; i < doomed.size(); ++i) // children, grandchildren...
		for (auto& it : nodes)
			if (it.second->parent == doomed[i])
				doomed.push_back(it.first);
	for (const shared_str& d : doomed)
	{
		auto it = nodes.find(d);
		if (it != nodes.end())
		{
			destroy(it->second);
			nodes.erase(it);
		}
	}
}

void HudPreview::Clear()
{
	xrCriticalSectionGuard g(lock);
	for (auto& it : nodes)
		destroy(it.second);
	nodes.clear();
	hide_hands = false;
}

bool HudPreview::Exists(LPCSTR id)
{
	xrCriticalSectionGuard g(lock);
	return !!find(id);
}

void HudPreview::SetTransform(LPCSTR id, const Fmatrix& m)
{
	xrCriticalSectionGuard g(lock);
	if (Node* n = find(id))
	{
		n->parent = nullptr;
		n->local = m;
		n->frame = u32(-1);
	}
}

void HudPreview::Attach(LPCSTR id, LPCSTR parent, const Fvector& pos, const Fvector& rot)
{
	xrCriticalSectionGuard g(lock);
	if (Node* n = find(id))
	{
		n->parent = parent;
		n->local = xyz(pos, rot);
		n->frame = u32(-1);
	}
}

void HudPreview::Detach(LPCSTR id)
{
	xrCriticalSectionGuard g(lock);
	if (Node* n = find(id))
	{
		n->parent = nullptr;
		n->local.identity();
	}
}

Fmatrix HudPreview::Transform(LPCSTR id)
{
	xrCriticalSectionGuard g(lock);
	Node* n = find(id);
	if (!n)
		return Fidentity;
	n->frame = u32(-1);
	return resolve(n);
}

void HudPreview::SetBoneVisible(LPCSTR id, LPCSTR bone, bool visible)
{
	xrCriticalSectionGuard g(lock);
	IKinematics* k = kinematics(find(id));
	const u16 b = k ? k->LL_BoneID(bone) : BI_NONE;
	if (b != BI_NONE && !!k->LL_GetBoneVisible(b) != visible)
		k->LL_SetBoneVisible(b, visible, TRUE);
}

void HudPreview::CopyBonesVisibility(LPCSTR id, CScriptGameObject* obj)
{
	if (!obj)
		return;
	IKinematics* src = object_kinematics(&obj->object());
	xrCriticalSectionGuard g(lock);
	IKinematics* dst = kinematics(find(id));
	if (!src || !dst)
		return;
	for (u16 b = 0; b < dst->LL_BoneCount(); ++b)
	{
		const u16 s = src->LL_BoneID(dst->LL_BoneName_dbg(b));
		if (s != BI_NONE && b != dst->LL_GetBoneRoot() && dst->LL_GetBoneVisible(b) != src->LL_GetBoneVisible(s))
			dst->LL_SetBoneVisible(b, src->LL_GetBoneVisible(s), FALSE);
	}
}

bool HudPreview::Play(LPCSTR id, LPCSTR motion, bool mix)
{
	xrCriticalSectionGuard g(lock);
	Node* n = find(id);
	IKinematicsAnimated* ka = n && n->visual ? n->visual->dcast_PKinematicsAnimated() : nullptr;
	if (!ka || !ka->LL_MotionsSlotCount())
		return false;
	const MotionID m = ka->ID_Cycle_Safe(motion);
	if (!m.valid())
		return false;
	ka->PlayCycle(m, mix);
	return true;
}

void HudPreview::FollowPose(LPCSTR id, CScriptGameObject* obj)
{
	xrCriticalSectionGuard g(lock);
	if (Node* n = find(id))
		n->pose_src = obj ? obj->ID() : u16(-1);
}

void HudPreview::HideHands(bool hide) { hide_hands = hide; }
bool HudPreview::HandsHidden() { return hide_hands && !nodes.empty(); }

void HudPreview::Update(const Fmatrix& hud_camera)
{
	xrCriticalSectionGuard g(lock);
	camera = hud_camera;
	for (auto& it : nodes)
		it.second->frame = u32(-1);
	for (auto& it : nodes)
	{
		Node* n = it.second;
		resolve(n); // parents first, through resolve()
		if (IKinematicsAnimated* ka = n->visual ? n->visual->dcast_PKinematicsAnimated() : nullptr)
			ka->UpdateTracks();
		if (IKinematics* k = kinematics(n))
		{
			k->CalculateBones_Invalidate();
			k->CalculateBones(TRUE);
			if (n->pose_src != u16(-1))
				if (IKinematics* src = object_kinematics(Level().Objects.net_Find(n->pose_src)))
					copy_pose(k, src);
		}
	}
}

void HudPreview::Render(IDSGraphManager* DM)
{
	xrCriticalSectionGuard g(lock);
	if (nodes.empty())
		return;
	const auto previous_role = DM->get_HUD_Role();
	DM->set_HUD_Role(IDSGraphManager::hud_prop);
	for (auto& it : nodes)
		if (it.second->visual)
			DM->add_Dynamic(it.second->visual, &it.second->world);
	DM->set_HUD_Role(previous_role);
}
