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
constexpr float kDegToRad = 0.017453292f;

// Left hand fingers: thumb, index .. little; root, middle, end joint.
const char* const kFingerBones[5][3] = {
	{"l_finger0", "l_finger01", "l_finger02"},
	{"bip01_l_finger1", "bip01_l_finger11", "bip01_l_finger12"},
	{"bip01_l_finger2", "bip01_l_finger21", "bip01_l_finger22"},
	{"bip01_l_finger3", "bip01_l_finger31", "bip01_l_finger32"},
	{"bip01_l_finger4", "bip01_l_finger41", "bip01_l_finger42"},
};

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

void HudHandContact::Request(float weight, float fade, bool grasp)
{
	clamp(weight, 0.f, 1.f);
	m_target = weight;
	m_speed = fade > EPS_L ? 1.f / fade : 1000.f;
	m_grasp = grasp;
}

void HudHandContact::Drive(float weight, bool grasp)
{
	m_drive = std::max(m_drive, std::min(weight, 1.f));
	m_drive_grasp = m_drive_grasp || grasp;
}

bool HudHandContact::SetParam(LPCSTR name, float value)
{
	static const std::pair<LPCSTR, float Params::*> fields[] = {
		{"cell", &Params::cell}, {"range", &Params::range}, {"touch", &Params::touch},
		{"palm_radius", &Params::palm_radius}, {"base_radius", &Params::base_radius},
		{"finger_radius", &Params::finger_radius}, {"tip_radius", &Params::tip_radius},
		{"tip_length", &Params::tip_length}, {"samples", &Params::samples}, {"max_push", &Params::max_push},
		{"push_response", &Params::push_response}, {"instant_push", &Params::instant_push},
		{"close_response", &Params::close_response}, {"open_response", &Params::open_response},
		{"curl_hysteresis", &Params::curl_hysteresis}, {"open_hysteresis", &Params::open_hysteresis},
		{"open_limit", &Params::open_limit}, {"curl_limit", &Params::curl_limit},
		{"search_steps", &Params::search_steps}, {"recheck", &Params::recheck}, {"deep", &Params::deep},
	};
	for (const auto& field : fields)
		if (!xr_strcmp(field.first, name))
		{
			if (!xr_strcmp(name, "cell"))
			{
				value = std::max(value, 0.002f);
				m_rebuild = m_rebuild || value != m_p.cell;
			}
			m_p.*field.second = value;
			return true;
		}
	return false;
}

void HudHandContact::Reset()
{
	m_hand_model = nullptr;
	m_weapon_model = nullptr;
	m_rebuild = true;
	m_push.set(0.f, 0.f, 0.f);
	std::fill(&m_curl[0][0], &m_curl[0][0] + kFingers * kJoints, 0.f);
	std::fill(&m_curl_target[0][0], &m_curl_target[0][0] + kFingers * kJoints, 0.f);
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
	m_rebuild = false;
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
	m_nx = iFloor((box.max.x - box.min.x) / m_p.cell) + 1;
	m_ny = iFloor((box.max.y - box.min.y) / m_p.cell) + 1;
	m_nz = iFloor((box.max.z - box.min.z) / m_p.cell) + 1;
	m_cells.resize(m_nx * m_ny * m_nz);
	for (u32 i = 0; i < m_tris.size(); ++i)
	{
		const Tri& t = m_tris[i];
		Fbox tb;
		tb.invalidate();
		tb.modify(t.a);
		tb.modify(t.b);
		tb.modify(t.c);
		const int x0 = iFloor((tb.min.x - m_min.x) / m_p.cell), x1 = iFloor((tb.max.x - m_min.x) / m_p.cell);
		const int y0 = iFloor((tb.min.y - m_min.y) / m_p.cell), y1 = iFloor((tb.max.y - m_min.y) / m_p.cell);
		const int z0 = iFloor((tb.min.z - m_min.z) / m_p.cell), z1 = iFloor((tb.max.z - m_min.z) / m_p.cell);
		for (int x = x0; x <= x1; ++x)
			for (int y = y0; y <= y1; ++y)
				for (int z = z0; z <= z1; ++z)
					m_cells[(x * m_ny + y) * m_nz + z].push_back(i);
	}
	m_stamp.assign(m_tris.size(), 0);
	m_visit = 0;
	Msg("[hud_hand_contact] weapon geometry: %u triangles, grid %dx%dx%d", u32(m_tris.size()), m_nx, m_ny, m_nz);
}

bool HudHandContact::Closest(const Fvector& p, float range, Hit& hit) const
{
	if (m_tris.empty())
		return false;
	const int x0 = std::max(0, iFloor((p.x - range - m_min.x) / m_p.cell)), x1 = std::min(m_nx - 1, iFloor((p.x + range - m_min.x) / m_p.cell));
	const int y0 = std::max(0, iFloor((p.y - range - m_min.y) / m_p.cell)), y1 = std::min(m_ny - 1, iFloor((p.y + range - m_min.y) / m_p.cell));
	const int z0 = std::max(0, iFloor((p.z - range - m_min.z) / m_p.cell)), z1 = std::min(m_nz - 1, iFloor((p.z + range - m_min.z) / m_p.cell));
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
	hit.point = best_point;
	if (d > 1e-5f)
		hit.normal.div(to_p, inside ? -d : d); // out of the weapon either way
	else
		hit.normal = best_tri->n;
	return true;
}

bool HudHandContact::Penetrates(const Fvector* points, const float* radii, int count, bool deep) const
{
	Hit hit;
	for (int i = 0; i < count; ++i)
	{
		if (Closest(points[i], radii[i] + m_p.touch, hit))
		{
			if (hit.distance < radii[i])
				return true;
		}
		else if (deep && Closest(points[i], m_p.range, hit) && hit.distance < 0.f)
			return true; // well inside a thick part, away from its surface
	}
	return false;
}

void HudHandContact::SolveFingers(IKinematics* K, const Fmatrix& to_item, float dt, float weight, bool grasp)
{
	const float close = 1.f - std::exp(-m_p.close_response * dt), open = 1.f - std::exp(-m_p.open_response * dt);
	// The full contact search is the expensive part: one finger per frame (all of them when contact starts).
	// Every frame each joint is checked at the angle it is about to take and searched again if it would go in.
	const int solve_finger = m_solve_all ? -1 : m_next_finger;
	m_next_finger = (m_next_finger + 1) % kFingers;
	m_solve_all = false;
	for (int f = 0; f < kFingers; ++f)
	{
		if (m_finger[f][0] == BI_NONE || m_finger[f][1] == BI_NONE || m_finger[f][2] == BI_NONE)
			continue;
		Fvector P[4]; // joints and the finger tip, left model space
		for (int j = 0; j < kJoints; ++j)
			P[j] = BonePos(K, m_finger[f][j]);
		P[3].sub(P[2], P[1]).mul(m_p.tip_length).add(P[2]);

		// Bend axis: the plane the finger already curls in; a straight finger bends toward the weapon.
		Fvector proximal, distal, axis;
		proximal.sub(P[1], P[0]);
		distal.sub(P[3], P[1]);
		axis.crossproduct(proximal, distal);
		if (axis.magnitude() < 1e-6f)
		{
			Hit hit;
			Fvector root;
			to_item.transform_tiny(root, P[0]);
			if (!Closest(root, m_p.range, hit))
				continue;
			Fvector toward = hit.normal;
			toward.invert(); // into the weapon, item space
			Fmatrix from_item;
			from_item.invert(to_item);
			from_item.transform_dir(toward);
			axis.crossproduct(proximal, toward);
			if (axis.magnitude() < 1e-6f)
				continue;
		}
		axis.normalize();

		for (int j = 0; j < kJoints; ++j)
		{
			// Do the joint's distal phalanges (samples per phalanx, item space) go into the weapon at a trial angle?
			auto blocked = [&](float angle, bool deep)
			{
				const Fmatrix turn = RotationAbout(P[j], axis, angle);
				const int per = std::clamp(int(m_p.samples), 1, 4);
				Fvector pts[12];
				float radii[12];
				int n = 0;
				for (int k = j; k < 3; ++k)
					for (int q = 1; q <= per; ++q)
					{
						Fvector at;
						at.lerp(P[k], P[k + 1], float(q) / per);
						turn.transform_tiny(at);
						to_item.transform_tiny(pts[n], at);
						radii[n++] = k == 2 && q == per ? m_p.tip_radius : m_p.finger_radius;
					}
				return Penetrates(pts, radii, n, deep);
			};
			// Boundary between a clear and a blocked angle, on the clear side.
			auto boundary = [&](float clear, float block)
			{
				for (int i = 0; i < int(m_p.search_steps); ++i)
				{
					const float mid = 0.5f * (clear + block);
					(blocked(mid, false) ? block : clear) = mid;
				}
				return clear;
			};

			const bool scheduled = solve_finger < 0 || solve_finger == f;
			const bool deep = m_p.deep > 0.f;
			if (scheduled || (m_p.recheck > 0.f && blocked(m_curl[f][j] * weight, false)))
			{
				float target = 0.f;
				if (blocked(0.f, deep))
					target = blocked(-(m_p.open_limit * kDegToRad), deep) ? -(m_p.open_limit * kDegToRad) : boundary(-(m_p.open_limit * kDegToRad), 0.f);
				else if (grasp && blocked((m_p.curl_limit * kDegToRad), false))
					target = boundary(0.f, (m_p.curl_limit * kDegToRad));
				// Hysteresis: small changes toward closing keep the old target, no finger flicker.
				const float change = target - m_curl_target[f][j];
				if (change < -m_p.open_hysteresis * kDegToRad || change > m_p.curl_hysteresis * kDegToRad || !scheduled)
					m_curl_target[f][j] = target;
				if (!scheduled)
					m_curl[f][j] = std::min(m_curl[f][j], target); // it was going in: no easing back out
			}

			const float goal = m_curl_target[f][j];
			m_curl[f][j] += (goal - m_curl[f][j]) * (goal < m_curl[f][j] ? open : close);
			const float angle = m_curl[f][j] * weight;
			if (_abs(angle) < 1e-5f)
				continue;
			const Fmatrix turn = RotationAbout(P[j], axis, angle);
			for (u16 b : m_moves[f][j])
				MoveBone(K, b, turn);
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
	const float weight = std::max(m_weight, m_drive);
	const bool grasp = (m_weight > 0.f && m_grasp) || m_drive_grasp;
	m_drive = 0.f;
	m_drive_grasp = false;
	if (weight > 0.f && m_effective <= 0.f)
		m_solve_all = true; // search every finger right away when contact starts
	m_effective = weight;
	if (weight <= 0.f)
	{
		m_push.set(0.f, 0.f, 0.f);
		std::fill(&m_curl[0][0], &m_curl[0][0] + kFingers * kJoints, 0.f);
		std::fill(&m_curl_target[0][0], &m_curl_target[0][0] + kFingers * kJoints, 0.f);
		return;
	}

	attachable_hud_item* item = hud.attached_item(0);
	if (!item || !item->m_model || hud.attached_item(1) || !hud.m_model_2)
		return;
	IKinematics* K = hud.m_model_2->dcast_PKinematics();
	if (!K || !ResolveBones(K))
		return;
	if (m_rebuild || item->m_model != m_weapon_model)
		BuildGeometry(item->m_model);
	if (m_tris.empty())
		return;

	// Left model space <-> weapon model space.
	Fmatrix item_inv, to_item, from_item;
	item_inv.invert(item->m_item_transform);
	to_item.mul_43(item_inv, hud.m_transform_2);
	from_item.invert(to_item);

	// Hand: push the palm and the finger roots out of the weapon.
	Fvector probes[5];
	float radii[5];
	const Fvector hand = BonePos(K, m_hand);
	int n = 0;
	if (m_finger[2][0] != BI_NONE)
	{
		probes[n].lerp(hand, BonePos(K, m_finger[2][0]), 0.55f);
		radii[n++] = m_p.palm_radius;
	}
	for (int f = 1; f < kFingers; ++f)
		if (m_finger[f][0] != BI_NONE)
		{
			probes[n] = BonePos(K, m_finger[f][0]);
			radii[n++] = m_p.base_radius;
		}
	for (int i = 0; i < n; ++i)
		to_item.transform_tiny(probes[i]);

	Fvector push = {0.f, 0.f, 0.f}; // item space
	for (int iteration = 0; iteration < 3; ++iteration)
	{
		float deepest = 0.f;
		Fvector step = {0.f, 0.f, 0.f};
		Hit hit;
		for (int i = 0; i < n; ++i)
		{
			Fvector p;
			p.add(probes[i], push);
			if (Closest(p, m_p.range, hit) && radii[i] - hit.distance > deepest)
			{
				deepest = radii[i] - hit.distance;
				step.mul(hit.normal, deepest);
			}
		}
		if (deepest <= 0.f)
			break;
		push.add(step);
	}
	if (push.magnitude() > m_p.max_push)
		push.normalize().mul(m_p.max_push);
	from_item.transform_dir(push);
	// With instant_push a deeper push is taken at once (no sinking in), a smaller one eases back.
	if (m_p.instant_push > 0.f && push.square_magnitude() >= m_push.square_magnitude())
		m_push = push;
	else
		m_push.lerp(m_push, push, 1.f - std::exp(-m_p.push_response * dt));

	Fvector shift;
	shift.mul(m_push, weight);
	if (shift.square_magnitude() > 1e-10f)
	{
		Fvector target;
		target.add(hand, shift);
		Fmatrix delta;
		delta.translate(shift);
		Bodycam::SolveHudArm(K, m_upperarm, m_forearm, m_twist, m_hand, target, BonePos(K, m_forearm), delta);
	}

	SolveFingers(K, to_item, dt, weight, grasp);
}
