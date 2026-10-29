#pragma once

class player_hud;
class IKinematics;

// Left hand contact with the weapon for script driven hand motion (game.hud_hand_contact):
// the left hand is pushed out of the weapon mesh with arm IK, and the fingers curl or open
// until they rest on its surface. Off by default; native weapon animations are left alone.
class HudHandContact
{
public:
	// weight 0..1 reached over fade seconds; grasp also wraps the fingers.
	void Request(float weight, float fade, bool grasp);
	// This frame only (a playing HUD pose clip): weight and grasp on top of Request.
	void Drive(float weight, bool grasp);
	// After the HUD pose is final for the frame (layers applied, weapon attached).
	void Apply(player_hud& hud);
	// The HUD models changed: bone ids and weapon geometry are stale.
	void Reset();

private:
	struct Tri
	{
		Fvector a, b, c, n;
	};
	struct Hit
	{
		float distance; // signed, negative inside the weapon
		Fvector normal; // out of the weapon at the closest point
		Fvector point;
	};
	enum
	{
		kFingers = 5,
		kJoints = 3
	};

	bool ResolveBones(IKinematics* hand);
	void BuildGeometry(IKinematics* weapon);
	bool Closest(const Fvector& p, float range, Hit& hit) const;
	// Signed distance minus radius of the most penetrating point (item space), large when clear.
	float Clearance(const Fvector* points, const float* radii, int count) const;
	void SolveFingers(IKinematics* K, const Fmatrix& to_item, float dt, float weight, bool grasp);

	float m_weight = 0.f, m_target = 0.f, m_speed = 0.f;
	bool m_grasp = true;
	float m_drive = 0.f; // this frame's clip weight
	bool m_drive_grasp = false;
	float m_effective = 0.f; // weight applied last frame

	// Left arm bones of the left HUD model.
	IKinematics* m_hand_model = nullptr;
	u16 m_upperarm = u16(-1), m_forearm = u16(-1), m_twist = u16(-1), m_hand = u16(-1);
	u16 m_finger[kFingers][kJoints];
	xr_vector<u16> m_moves[kFingers][kJoints]; // the joint and every bone below it

	// Weapon triangles in its model space, bucketed in a uniform grid.
	IKinematics* m_weapon_model = nullptr;
	bool m_rebuild = true;
	xr_vector<Tri> m_tris;
	xr_vector<xr_vector<u32>> m_cells;
	Fvector m_min;
	int m_nx = 0, m_ny = 0, m_nz = 0;
	mutable xr_vector<u32> m_stamp; // per triangle, visit marks for one query
	mutable u32 m_visit = 0;

	Fvector m_push = {0.f, 0.f, 0.f}; // smoothed hand push, left model space
	float m_curl[kFingers][kJoints] = {}; // smoothed extra joint angles, radians
};
