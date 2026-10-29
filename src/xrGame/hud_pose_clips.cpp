#include "stdafx.h"
#include "hud_pose_clips.h"
#include "player_hud.h"
#include "bodycam_hud_arms.h"
#include "../Include/xrRender/Kinematics.h"
#include "../xrEngine/bone.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr float kSameKey = 0.0005f; // s; keys closer than this are the same key
constexpr float kDegToRad = 0.017453292f;

float MinJerk(float t) { return t * t * t * (10.f + t * (-15.f + 6.f * t)); }

bool IsBelow(IKinematics* K, u16 bone, u16 root)
{
	for (u16 b = bone; b != BI_NONE; b = K->LL_GetData(b).GetParentID())
		if (b == root)
			return true;
	return false;
}

int Depth(IKinematics* K, u16 bone)
{
	int depth = 0;
	for (u16 b = K->LL_GetData(bone).GetParentID(); b != BI_NONE; b = K->LL_GetData(b).GetParentID())
		++depth;
	return depth;
}

// The bone and everything below it, moved by delta (model space).
void MoveHierarchy(IKinematics* K, u16 root, const Fmatrix& delta)
{
	for (u16 b = 0; b < K->LL_BoneCount(); ++b)
	{
		if (!IsBelow(K, b, root))
			continue;
		CBoneInstance& bone = K->LL_GetBoneInstance(b);
		Fmatrix moved;
		moved.mul_43(delta, bone.mTransform);
		bone.mTransform.set(moved);
		bone.mRenderTransform.mul_43(bone.mTransform, K->LL_GetData(b).m2b_transform);
	}
}

// Local offset (weighted) as a matrix: rotation x pitch, y yaw, z roll in degrees.
Fmatrix Offset(const Fvector& pos, const Fvector& rot, float w)
{
	Fmatrix m;
	m.setHPB(rot.y * w * kDegToRad, rot.x * w * kDegToRad, rot.z * w * kDegToRad);
	m.c.mul(pos, w);
	return m;
}

// frame * offset * frame^-1: the offset applied in a bone's own frame, as a model space delta.
Fmatrix InFrame(const Fmatrix& frame, const Fmatrix& offset)
{
	Fmatrix inv, d;
	inv.invert(frame);
	d.mul_43(frame, offset);
	d.mulB_43(inv);
	return d;
}

bool IsLeftBone(LPCSTR name) { return !strncmp(name, "l_", 2) || !strncmp(name, "bip01_l_", 8); }
} // namespace

float HudPoseClips::Clip::Length() const
{
	float length = 0.f;
	for (const Track& t : tracks)
		if (!t.keys.empty())
			length = std::max(length, t.keys.back().t);
	return length;
}

HudPoseClips::Clip* HudPoseClips::Find(LPCSTR name) const
{
	for (Clip* c : m_clips)
		if (!xr_strcmp(c->name, name))
			return c;
	return nullptr;
}

HudPoseClips::Clip& HudPoseClips::Get(LPCSTR name)
{
	if (Clip* c = Find(name))
		return *c;
	Clip* c = xr_new<Clip>();
	c->name = name;
	m_clips.push_back(c);
	return *c;
}

HudPoseClips::Track& HudPoseClips::GetTrack(Clip& clip, LPCSTR name)
{
	for (Track& t : clip.tracks)
		if (!xr_strcmp(t.name, name))
			return t;
	clip.tracks.push_back(Track());
	clip.tracks.back().name = name;
	return clip.tracks.back();
}

void HudPoseClips::Insert(Track& track, const Key_& key)
{
	for (Key_& k : track.keys)
		if (_abs(k.t - key.t) < kSameKey)
		{
			k = key;
			return;
		}
	track.keys.insert(std::upper_bound(track.keys.begin(), track.keys.end(), key,
		[](const Key_& a, const Key_& b) { return a.t < b.t; }), key);
}

// Cubic Hermite through the keys, Catmull-Rom tangents over uneven spacing, flat at the ends.
HudPoseClips::Key_ HudPoseClips::Sample(const Track& track, float t)
{
	const xr_vector<Key_>& k = track.keys;
	Key_ out = {t, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, 0.f};
	if (k.empty())
		return out;
	if (t <= k.front().t || k.size() == 1)
		return k.front();
	if (t >= k.back().t)
		return k.back();
	u32 i = 0;
	while (k[i + 1].t < t)
		++i;
	const Key_ &a = k[i], &b = k[i + 1];
	const float h = std::max(b.t - a.t, 1e-6f), s = (t - a.t) / h;
	const float h00 = 2 * s * s * s - 3 * s * s + 1, h10 = s * s * s - 2 * s * s + s, h01 = -2 * s * s * s + 3 * s * s, h11 = s * s * s - s * s;
	auto tangent = [&](u32 j, auto field) -> float
	{
		if (j == 0 || j + 1 >= k.size())
			return 0.f;
		return (field(k[j + 1]) - field(k[j - 1])) / std::max(k[j + 1].t - k[j - 1].t, 1e-6f);
	};
	auto curve = [&](auto field)
	{
		return h00 * field(a) + h10 * h * tangent(i, field) + h01 * field(b) + h11 * h * tangent(i + 1, field);
	};
	out.pos.set(curve([](const Key_& q) { return q.pos.x; }), curve([](const Key_& q) { return q.pos.y; }), curve([](const Key_& q) { return q.pos.z; }));
	out.rot.set(curve([](const Key_& q) { return q.rot.x; }), curve([](const Key_& q) { return q.rot.y; }), curve([](const Key_& q) { return q.rot.z; }));
	out.value = curve([](const Key_& q) { return q.value; });
	return out;
}

void HudPoseClips::Key(LPCSTR clip, LPCSTR track, float time, const Fvector& pos, const Fvector& rot)
{
	Insert(GetTrack(Get(clip), track), Key_{time, pos, rot, 0.f});
}

void HudPoseClips::Value(LPCSTR clip, LPCSTR track, float time, float value)
{
	Insert(GetTrack(Get(clip), track), Key_{time, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, value});
}

void HudPoseClips::RemoveKey(LPCSTR clip, LPCSTR track, float time)
{
	if (Clip* c = Find(clip))
		for (Track& t : c->tracks)
			if (!xr_strcmp(t.name, track))
				t.keys.erase(std::remove_if(t.keys.begin(), t.keys.end(), [&](const Key_& k) { return _abs(k.t - time) < kSameKey; }),
					t.keys.end());
}

void HudPoseClips::Remove(LPCSTR clip)
{
	for (auto it = m_clips.begin(); it != m_clips.end(); ++it)
		if (!xr_strcmp((*it)->name, clip))
		{
			xr_delete(*it);
			m_clips.erase(it);
			return;
		}
}

void HudPoseClips::Play(LPCSTR clip, float speed, bool loop, float fade)
{
	Clip* c = Find(clip);
	if (!c)
	{
		Msg("! [hud_clip] no clip [%s]", clip);
		return;
	}
	c->time = 0.f;
	c->speed = speed;
	c->loop = loop;
	c->paused = false;
	c->active = true;
	c->target = 1.f;
	c->fade_speed = fade > EPS_L ? 1.f / fade : 1000.f;
}

void HudPoseClips::Stop(LPCSTR clip, float fade)
{
	if (Clip* c = Find(clip))
	{
		c->target = 0.f;
		c->fade_speed = fade > EPS_L ? 1.f / fade : 1000.f;
	}
}

void HudPoseClips::Pause(LPCSTR clip, bool paused)
{
	if (Clip* c = Find(clip))
	{
		c->paused = paused;
		if (paused && !c->active) // holding a pose for editing
		{
			c->active = true;
			c->target = 1.f;
			c->fade_speed = 1000.f;
		}
	}
}

void HudPoseClips::SetTime(LPCSTR clip, float time)
{
	if (Clip* c = Find(clip))
		c->time = std::max(time, 0.f);
}

float HudPoseClips::Time(LPCSTR clip) const
{
	const Clip* c = Find(clip);
	return c ? c->time : 0.f;
}

float HudPoseClips::Length(LPCSTR clip) const
{
	const Clip* c = Find(clip);
	return c ? c->Length() : 0.f;
}

bool HudPoseClips::Playing(LPCSTR clip) const
{
	const Clip* c = Find(clip);
	return c && c->active;
}

void HudPoseClips::Reset()
{
	for (Clip*& c : m_clips)
		xr_delete(c);
	m_clips.clear();
}

void HudPoseClips::Apply(player_hud& hud)
{
	IKinematics* R = hud.m_model ? hud.m_model->dcast_PKinematics() : nullptr;
	IKinematics* L = hud.m_model_2 ? hud.m_model_2->dcast_PKinematics() : nullptr;
	if (!R || !L)
		return;
	m_base = hud.m_transform;
	const float dt = std::min(Device.fTimeDelta, 0.05f);
	const u16 r_lead = R->LL_BoneID("lead_gun"), l_lead = L->LL_BoneID("lead_gun");

	for (Clip* c : m_clips)
	{
		if (!c->active)
			continue;
		if (c->weight < c->target)
			c->weight = std::min(c->target, c->weight + dt * c->fade_speed);
		else if (c->weight > c->target)
			c->weight = std::max(c->target, c->weight - dt * c->fade_speed);
		const float length = c->Length();
		if (!c->paused)
		{
			c->time += dt * c->speed;
			if (c->time >= length)
			{
				if (c->loop && length > 0.f)
					c->time = std::fmod(c->time, length);
				else
				{
					c->time = length;
					c->target = 0.f;
				}
			}
		}
		if (c->weight <= 0.f && c->target <= 0.f)
		{
			c->active = false;
			continue;
		}
		const float w = MinJerk(c->weight);

		// Rigid tracks first, then bones parent first so children see their parents' new frames.
		const Track* hand_ik = nullptr;
		xr_vector<std::pair<int, const Track*>> bones;
		for (const Track& track : c->tracks)
		{
			if (track.keys.empty())
				continue;
			LPCSTR name = track.name.c_str();
			if (!xr_strcmp(name, "contact"))
				hud.m_hand_contact.Drive(Sample(track, c->time).value * w, false);
			else if (!xr_strcmp(name, "grasp"))
				hud.m_hand_contact.Drive(0.f, Sample(track, c->time).value * w > 0.5f);
			else if (!xr_strcmp(name, "l_hand_ik"))
				hand_ik = &track;
			else if (!xr_strcmp(name, "gun"))
			{
				const Key_ k = Sample(track, c->time);
				const Fmatrix offset = Offset(k.pos, k.rot, w);
				hud.m_transform.mulB_43(r_lead != BI_NONE ? InFrame(R->LL_GetTransform(r_lead), offset) : offset);
				hud.m_transform_2.mulB_43(l_lead != BI_NONE ? InFrame(L->LL_GetTransform(l_lead), offset) : offset);
			}
			else
			{
				IKinematics* K = IsLeftBone(name) ? L : R;
				const u16 id = K->LL_BoneID(name);
				if (id != BI_NONE)
					bones.push_back({Depth(K, id), &track});
			}
		}

		if (hand_ik && l_lead != BI_NONE)
		{
			const u16 up = L->LL_BoneID("l_upperarm"), fore = L->LL_BoneID("l_forearm"), hand = L->LL_BoneID("l_hand");
			if (up != BI_NONE && fore != BI_NONE && hand != BI_NONE)
			{
				const Key_ k = Sample(*hand_ik, c->time);
				const Fmatrix delta = InFrame(L->LL_GetTransform(l_lead), Offset(k.pos, k.rot, w));
				Fvector target;
				delta.transform_tiny(target, L->LL_GetTransform(hand).c);
				Bodycam::SolveHudArm(L, up, fore, L->LL_BoneID("l_forearm_twist"), hand, target,
					L->LL_GetTransform(fore).c, delta);
			}
		}

		std::sort(bones.begin(), bones.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
		for (const auto& entry : bones)
		{
			LPCSTR name = entry.second->name.c_str();
			IKinematics* K = IsLeftBone(name) ? L : R;
			const u16 id = K->LL_BoneID(name);
			const Key_ k = Sample(*entry.second, c->time);
			MoveHierarchy(K, id, InFrame(K->LL_GetTransform(id), Offset(k.pos, k.rot, w)));
		}
	}
}

bool HudPoseClips::CaptureStart(LPCSTR file, LPCSTR clip)
{
	CaptureStop();
	string_path path;
	FS.update_path(path, "$game_anims$", file);
	m_capture = fopen(path, "w");
	if (!m_capture)
	{
		Msg("! [hud_clip] can't write capture %s", path);
		return false;
	}
	m_capture_clip = clip;
	m_capture_header = false;
	Msg("[hud_clip] capturing [%s] to %s", clip, path);
	return true;
}

void HudPoseClips::CaptureStop()
{
	if (m_capture)
		fclose(m_capture);
	m_capture = nullptr;
}

void HudPoseClips::Capture(player_hud& hud)
{
	if (!m_capture)
		return;
	IKinematics* R = hud.m_model ? hud.m_model->dcast_PKinematics() : nullptr;
	IKinematics* L = hud.m_model_2 ? hud.m_model_2->dcast_PKinematics() : nullptr;
	if (!R || !L)
		return;
	const u16 count = R->LL_BoneCount();
	if (!m_capture_header)
	{
		fprintf(m_capture, "hud_capture 1\nbones %u\n", u32(count));
		for (u16 b = 0; b < count; ++b)
		{
			const u16 parent = R->LL_GetData(b).GetParentID();
			fprintf(m_capture, "%s %s\n", R->LL_BoneName_dbg(b), parent == BI_NONE ? "-" : R->LL_BoneName_dbg(parent));
		}
		m_capture_header = true;
	}
	const Clip* c = Find(m_capture_clip.c_str());
	fprintf(m_capture, "frame %.5f\n", c ? c->time : 0.f);

	Fmatrix inv_base;
	inv_base.invert(m_base);
	const u16 l_root = R->LL_BoneID("l_clavicle");
	xr_vector<Fmatrix> model(count);
	for (u16 b = 0; b < count; ++b)
	{
		const bool left = l_root != BI_NONE && b < L->LL_BoneCount() && IsBelow(R, b, l_root);
		Fmatrix world;
		world.mul_43(left ? hud.m_transform_2 : hud.m_transform, (left ? L : R)->LL_GetTransform(b));
		model[b].mul_43(inv_base, world);
	}
	for (u16 b = 0; b < count; ++b)
	{
		const u16 parent = R->LL_GetData(b).GetParentID();
		Fmatrix local = model[b];
		if (parent != BI_NONE)
		{
			Fmatrix inv_parent;
			inv_parent.invert(model[parent]);
			local.mul_43(inv_parent, model[b]);
		}
		fprintf(m_capture, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", local.i.x, local.i.y, local.i.z,
			local.j.x, local.j.y, local.j.z, local.k.x, local.k.y, local.k.z, local.c.x, local.c.y, local.c.z);
	}
}
