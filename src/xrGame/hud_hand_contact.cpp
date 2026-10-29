#include "stdafx.h"
#include "hud_hand_contact.h"
#include "player_hud.h"
#include "bodycam_hud_arms.h"
#include "../Include/xrRender/Kinematics.h"
#include "../xrEngine/bone.h"
#include "../xrEngine/EnnumerateVertices.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr float kCell = 0.01f;          // weapon mesh grid, m
constexpr float kRange = 0.03f;         // farthest surface a query looks for, m
constexpr float kPalmRadius = 0.014f;
constexpr float kRootRadius = 0.009f;   // finger roots
constexpr float kFingerRadius = 0.008f;
constexpr float kTipRadius = 0.007f;
constexpr float kTipLength = 0.9f;      // end phalanx as a share of the middle one
constexpr int kSamples = 3;             // per phalanx
constexpr float kMaxPush = 0.06f;
constexpr float kOpenLimit = 0.5235988f; // 30 deg
constexpr int kSearchSteps = 12;

// Left hand fingers: thumb, index .. little; root, middle, end joint.
const char* const kFingerBones[5][3] = {
	{"l_finger0", "l_finger01", "l_finger02"},
	{"bip01_l_finger1", "bip01_l_finger11", "bip01_l_finger12"},
	{"bip01_l_finger2", "bip01_l_finger21", "bip01_l_finger22"},
	{"bip01_l_finger3", "bip01_l_finger31", "bip01_l_finger32"},
	{"bip01_l_finger4", "bip01_l_finger41", "bip01_l_finger42"},
};

float MinJerk(float t) { return t * t * t * (10.f + t * (-15.f + 6.f * t)); }

Fvector BonePos(IKinematics* K, u16 id) { return K->LL_GetBoneInstance(id).mTransform.c; }

bool IsBelow(IKinematics* K, u16 bone, u16 root)
{
	for (u16 b = bone; b != BI_NONE; b = K->LL_GetData(b).GetParentID())
		if (b == root)
			return true;
	return false;
}

void MoveBone(IKinematics* K, u16 id, const Fmatrix& delta)
{
	CBoneInstance& bone = K->LL_GetBoneInstance(id);
	Fmatrix moved;
	moved.mul_43(delta, bone.mTransform);
	bone.mTransform.set(moved);
	bone.mRenderTransform.mul_43(bone.mTransform, K->LL_GetData(id).m2b_transform);
}

Fmatrix RotationAbout(const Fvector& pivot, const Fvector& axis, float angle)
{
	Fmatrix m;
	m.rotation(axis, angle);
	Fvector turned;
	m.transform_dir(turned, pivot);
	m.c.sub(pivot, turned);
	return m;
}

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
Fvector ClosestOnTri(const Fvector& p, const Fvector& a, const Fvector& b, const Fvector& c)
{
	Fvector ab, ac, ap, r;
	ab.sub(b, a); ac.sub(c, a); ap.sub(p, a);
	const float d1 = ab.dotproduct(ap), d2 = ac.dotproduct(ap);
	if (d1 <= 0.f && d2 <= 0.f) return a;
	Fvector bp; bp.sub(p, b);
	const float d3 = ab.dotproduct(bp), d4 = ac.dotproduct(bp);
	if (d3 >= 0.f && d4 <= d3) return b;
	const float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) return r.mad(a, ab, d1 / (d1 - d3));
	Fvector cp; cp.sub(p, c);
	const float d5 = ab.dotproduct(cp), d6 = ac.dotproduct(cp);
	if (d6 >= 0.f && d5 <= d6) return c;
	const float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) return r.mad(a, ac, d2 / (d2 - d6));
	const float va = d3 * d6 - d5 * d4;
	if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
	{
		Fvector bc; bc.sub(c, b);
		return r.mad(b, bc, (d4 - d3) / ((d4 - d3) + (d5 - d6)));
	}
	const float denom = 1.f / (va + vb + vc);
	r.mad(a, ab, vb * denom);
	return r.mad(ac, vc * denom);
}

struct TriCollector : public SEnumVerticesCallback
{
	const Fmatrix& skin; // current pose of the enumerated bone
	xr_vector<Fvector>& out;
	TriCollector(const Fmatrix& s, xr_vector<Fvector>& o) : skin(s), out(o) {}
	void operator()(const Fvector& p) override
	{
		Fvector q;
		skin.transform_tiny(q, p);
		out.push_back(q);
	}
};
} // namespace

void HudHandContact::Request(float weight, float fade)
{
	clamp(weight, 0.f, 1.f);
	if (weight > 0.f && m_weight <= 0.f)
		m_solve = true; // check the hand once, as it is when contact turns on
	m_target = weight;
	m_speed = fade > EPS_L ? 1.f / fade : 1000.f;
}

void HudHandContact::Reset()
{
	m_hand_model = nullptr;
	m_weapon_model = nullptr;
	m_weight = m_target = 0.f;
}

bool HudHandContact::ResolveBones(IKinematics* K)
{
	if (K == m_hand_model)
		return m_hand != BI_NONE;
	m_hand_model = K;
	m_upperarm = K->LL_BoneID("l_upperarm");
	m_forearm = K->LL_BoneID("l_forearm");
	m_twist = K->LL_BoneID("l_forearm_twist");
	m_hand = K->LL_BoneID("l_hand");
	for (int f = 0; f < kFingers; ++f)
		for (int j = 0; j < kJoints; ++j)
		{
			m_finger[f][j] = K->LL_BoneID(kFingerBones[f][j]);
			m_moves[f][j].clear();
			if (m_finger[f][j] == BI_NONE)
				continue;
			for (u16 b = 0; b < K->LL_BoneCount(); ++b)
				if (IsBelow(K, b, m_finger[f][j]))
					m_moves[f][j].push_back(b);
		}
	if (m_upperarm == BI_NONE || m_forearm == BI_NONE || m_hand == BI_NONE)
		m_hand = BI_NONE;
	return m_hand != BI_NONE;
}

void HudHandContact::BuildGeometry(IKinematics* W)
{
	m_weapon_model = W;
	m_tris.clear();
	m_cells.clear();

	xr_vector<Fvector> verts;
	for (u16 b = 0; b < W->LL_BoneCount(); ++b)
	{
		const u32 from = verts.size();
		TriCollector collect(W->LL_GetBoneInstance(b).mRenderTransform, verts);
		W->EnumBoneVertices(collect, b);
		verts.resize(from + (verts.size() - from) / 3 * 3); // whole faces only
	}

	Fbox box;
	box.invalidate();
	float volume = 0.f;
	for (u32 i = 0; i + 2 < verts.size(); i += 3)
	{
		Tri t{verts[i], verts[i + 1], verts[i + 2]};
		Fvector e1, e2;
		e1.sub(t.b, t.a);
		e2.sub(t.c, t.a);
		t.n.crossproduct(e1, e2);
		const float area2 = t.n.magnitude();
		if (area2 < 1e-10f)
			continue;
		t.n.div(area2);
		volume += t.a.dotproduct(Fvector().crossproduct(t.b, t.c));
		box.modify(t.a);
		box.modify(t.b);
		box.modify(t.c);
		m_tris.push_back(t);
	}
	if (m_tris.empty())
	{
		Msg("! [hud_hand_contact] weapon model has no readable geometry");
		return;
	}
	if (volume < 0.f) // inward winding: point the normals out
		for (Tri& t : m_tris)
			t.n.invert();

	m_min = box.min;
	m_nx = iFloor((box.max.x - box.min.x) / kCell) + 1;
	m_ny = iFloor((box.max.y - box.min.y) / kCell) + 1;
	m_nz = iFloor((box.max.z - box.min.z) / kCell) + 1;
	m_cells.resize(m_nx * m_ny * m_nz);
	for (u32 i = 0; i < m_tris.size(); ++i)
	{
		const Tri& t = m_tris[i];
		Fbox tb;
		tb.invalidate();
		tb.modify(t.a);
		tb.modify(t.b);
		tb.modify(t.c);
		const int x0 = iFloor((tb.min.x - m_min.x) / kCell), x1 = iFloor((tb.max.x - m_min.x) / kCell);
		const int y0 = iFloor((tb.min.y - m_min.y) / kCell), y1 = iFloor((tb.max.y - m_min.y) / kCell);
		const int z0 = iFloor((tb.min.z - m_min.z) / kCell), z1 = iFloor((tb.max.z - m_min.z) / kCell);
		for (int x = x0; x <= x1; ++x)
			for (int y = y0; y <= y1; ++y)
				for (int z = z0; z <= z1; ++z)
					m_cells[(x * m_ny + y) * m_nz + z].push_back(i);
	}
	m_stamp.assign(m_tris.size(), 0);
	m_visit = 0;
	Msg("[hud_hand_contact] weapon geometry: %u triangles", u32(m_tris.size()));
}

bool HudHandContact::Closest(const Fvector& p, float range, Hit& hit) const
{
	if (m_tris.empty())
		return false;
	const int x0 = std::max(0, iFloor((p.x - range - m_min.x) / kCell)), x1 = std::min(m_nx - 1, iFloor((p.x + range - m_min.x) / kCell));
	const int y0 = std::max(0, iFloor((p.y - range - m_min.y) / kCell)), y1 = std::min(m_ny - 1, iFloor((p.y + range - m_min.y) / kCell));
	const int z0 = std::max(0, iFloor((p.z - range - m_min.z) / kCell)), z1 = std::min(m_nz - 1, iFloor((p.z + range - m_min.z) / kCell));
	if (x0 > x1 || y0 > y1 || z0 > z1)
		return false;
	if (++m_visit == 0)
	{
		std::fill(m_stamp.begin(), m_stamp.end(), 0);
		m_visit = 1;
	}
	float best = range * range;
	const Tri* best_tri = nullptr;
	Fvector best_point;
	for (int x = x0; x <= x1; ++x)
		for (int y = y0; y <= y1; ++y)
			for (int z = z0; z <= z1; ++z)
				for (u32 i : m_cells[(x * m_ny + y) * m_nz + z])
				{
					if (m_stamp[i] == m_visit)
						continue;
					m_stamp[i] = m_visit;
					const Tri& t = m_tris[i];
					const Fvector q = ClosestOnTri(p, t.a, t.b, t.c);
					const float d2 = q.distance_to_sqr(p);
					if (d2 < best)
					{
						best = d2;
						best_tri = &t;
						best_point = q;
					}
				}
	if (!best_tri)
		return false;
	Fvector to_p;
	to_p.sub(p, best_point);
	const float d = _sqrt(best);
	const bool inside = to_p.dotproduct(best_tri->n) < 0.f;
	hit.distance = inside ? -d : d;
	if (d > 1e-5f)
		hit.normal.div(to_p, inside ? -d : d); // out of the weapon either way
	else
		hit.normal = best_tri->n;
	return true;
}

bool HudHandContact::Inside(const Fvector& p, float radius) const
{
	Hit hit;
	return Closest(p, kRange, hit) && hit.distance < radius;
}

// Bend axis: the plane the finger already curls in; a straight finger bends toward the weapon.
bool HudHandContact::FingerAxis(IKinematics* K, int f, const Fmatrix& to_item, Fvector& axis) const
{
	const Fvector p0 = BonePos(K, m_finger[f][0]), p1 = BonePos(K, m_finger[f][1]), p2 = BonePos(K, m_finger[f][2]);
	Fvector proximal, distal;
	proximal.sub(p1, p0);
	distal.sub(p2, p1);
	axis.crossproduct(proximal, distal);
	if (axis.magnitude() < 1e-6f)
	{
		Hit hit;
		Fvector root;
		to_item.transform_tiny(root, p0);
		if (!Closest(root, kRange, hit))
			return false;
		Fmatrix from_item;
		from_item.invert(to_item);
		Fvector toward = hit.normal;
		toward.invert();
		from_item.transform_dir(toward);
		axis.crossproduct(proximal, toward);
		if (axis.magnitude() < 1e-6f)
			return false;
	}
	axis.normalize();
	return true;
}

void HudHandContact::Solve(IKinematics* K, const Fmatrix& to_item)
{
	std::fill(&m_open[0][0], &m_open[0][0] + kFingers * kJoints, 0.f);
	m_push.set(0.f, 0.f, 0.f);

	// Hand: push the palm and the finger roots out of the weapon.
	Fvector probes[5];
	float radii[5];
	int n = 0;
	const Fvector hand = BonePos(K, m_hand);
	if (m_finger[2][0] != BI_NONE)
	{
		probes[n].lerp(hand, BonePos(K, m_finger[2][0]), 0.55f);
		radii[n++] = kPalmRadius;
	}
	for (int f = 1; f < kFingers; ++f)
		if (m_finger[f][0] != BI_NONE)
		{
			probes[n] = BonePos(K, m_finger[f][0]);
			radii[n++] = kRootRadius;
		}
	for (int i = 0; i < n; ++i)
		to_item.transform_tiny(probes[i]);
	Fvector push = {0.f, 0.f, 0.f}; // item space
	for (int iteration = 0; iteration < 6; ++iteration)
	{
		float deepest = 0.f;
		Fvector step = {0.f, 0.f, 0.f};
		Hit hit;
		for (int i = 0; i < n; ++i)
		{
			Fvector p;
			p.add(probes[i], push);
			if (Closest(p, kRange, hit) && radii[i] - hit.distance > deepest)
			{
				deepest = radii[i] - hit.distance;
				step.mul(hit.normal, deepest);
			}
		}
		if (deepest <= 0.f)
			break;
		push.add(step);
	}
	if (push.magnitude() > kMaxPush)
		push.normalize().mul(kMaxPush);
	Fmatrix from_item;
	from_item.invert(to_item);
	from_item.transform_dir(m_push, push);

	// Fingers, on the pushed hand: each joint from the root opens just enough to keep its
	// distal phalanges out of the weapon. Nothing closes.
	for (int f = 0; f < kFingers; ++f)
	{
		if (m_finger[f][0] == BI_NONE || m_finger[f][1] == BI_NONE || m_finger[f][2] == BI_NONE)
			continue;
		Fvector axis;
		if (!FingerAxis(K, f, to_item, axis))
			continue;
		Fvector P[4];
		for (int j = 0; j < kJoints; ++j)
			P[j].add(BonePos(K, m_finger[f][j]), m_push);
		P[3].sub(P[2], P[1]).mul(kTipLength).add(P[2]);

		for (int j = 0; j < kJoints; ++j)
		{
			auto blocked = [&](float angle)
			{
				const Fmatrix turn = RotationAbout(P[j], axis, angle);
				for (int k = j; k < 3; ++k)
					for (int q = 1; q <= kSamples; ++q)
					{
						Fvector at, item;
						at.lerp(P[k], P[k + 1], float(q) / kSamples);
						turn.transform_tiny(at);
						to_item.transform_tiny(item, at);
						if (Inside(item, k == 2 && q == kSamples ? kTipRadius : kFingerRadius))
							return true;
					}
				return false;
			};
			if (!blocked(0.f))
				continue;
			float clear = -kOpenLimit, block = 0.f;
			if (!blocked(clear))
				for (int i = 0; i < kSearchSteps; ++i)
				{
					const float mid = 0.5f * (clear + block);
					(blocked(mid) ? block : clear) = mid;
				}
			m_open[f][j] = clear;
			const Fmatrix turn = RotationAbout(P[j], axis, clear);
			for (int k = j + 1; k < 4; ++k)
				turn.transform_tiny(P[k]);
		}
	}
}

void HudHandContact::Apply(player_hud& hud)
{
	const float dt = std::min(Device.fTimeDelta, 0.05f);
	if (m_weight < m_target)
		m_weight = std::min(m_target, m_weight + dt * m_speed);
	else if (m_weight > m_target)
		m_weight = std::max(m_target, m_weight - dt * m_speed);
	if (m_weight <= 0.f)
		return;

	attachable_hud_item* item = hud.attached_item(0);
	if (!item || !item->m_model || hud.attached_item(1) || !hud.m_model_2)
		return;
	IKinematics* K = hud.m_model_2->dcast_PKinematics();
	if (!K || !ResolveBones(K))
		return;
	if (item->m_model != m_weapon_model)
	{
		BuildGeometry(item->m_model);
		m_solve = true;
	}
	if (m_tris.empty())
		return;

	if (m_solve)
	{
		Fmatrix item_inv, to_item;
		item_inv.invert(item->m_item_transform);
		to_item.mul_43(item_inv, hud.m_transform_2);
		Solve(K, to_item);
		m_solve = false;
	}

	const float w = MinJerk(m_weight);
	Fvector shift;
	shift.mul(m_push, w);
	if (shift.square_magnitude() > 1e-10f)
	{
		Fvector target;
		target.add(BonePos(K, m_hand), shift);
		Fmatrix delta;
		delta.translate(shift);
		Bodycam::SolveHudArm(K, m_upperarm, m_forearm, m_twist, m_hand, target, BonePos(K, m_forearm), delta);
	}

	Fmatrix item_inv, to_item;
	item_inv.invert(item->m_item_transform);
	to_item.mul_43(item_inv, hud.m_transform_2);
	for (int f = 0; f < kFingers; ++f)
	{
		Fvector axis;
		bool have_axis = false;
		for (int j = 0; j < kJoints; ++j)
		{
			const float angle = m_open[f][j] * w;
			if (_abs(angle) < 1e-5f)
				continue;
			if (!have_axis && !(have_axis = FingerAxis(K, f, to_item, axis)))
				break;
			const Fmatrix turn = RotationAbout(BonePos(K, m_finger[f][j]), axis, angle);
			for (u16 b : m_moves[f][j])
				MoveBone(K, b, turn);
		}
	}
}
