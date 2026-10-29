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
	// Tuning without rebuilding the engine (game.hud_hand_contact_param, configs\plugins\hud_hand_contact.ltx).
	// False for an unknown name.
	bool SetParam(LPCSTR name, float value);

	struct Params
	{
		float cell = 0.01f;           // weapon mesh grid, m (rebuilds the grid)
		float range = 0.025f;         // hand search distance, m
		float touch = 0.004f;         // fingers look this far past their radius, m
		float palm_radius = 0.014f;
		float base_radius = 0.009f;   // finger roots
		float finger_radius = 0.0075f;
		float tip_radius = 0.0065f;
		float tip_length = 0.8f;      // end phalanx length as a share of the middle one
		float samples = 2.f;          // samples per phalanx
		float max_push = 0.08f;       // m
		float push_response = 18.f;   // 1/s
		float instant_push = 0.f;     // 1: a deeper hand push is applied at once
		float close_response = 7.f;   // 1/s, fingers closing onto the weapon
		float open_response = 7.f;    // 1/s, fingers backing out of it
		float curl_hysteresis = 5.2f; // deg: a closing target must change this much
		float open_hysteresis = 1.7f; // deg: an opening target must change this much
		float open_limit = 20.f;      // deg
		float curl_limit = 50.f;      // deg
		float search_steps = 6.f;
		float recheck = 0.f;          // 1: check every joint every frame, search again at once if it goes in
		float deep = 0.f;             // 1: also find finger points well inside thick parts
	};

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
	// Any point (item space) closer to the weapon surface than its radius; deep also finds points
	// well inside thick parts (a longer search).
	bool Penetrates(const Fvector* points, const float* radii, int count, bool deep) const;
	Params m_p;
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
	float m_curl_target[kFingers][kJoints] = {}; // last contact search result
	int m_next_finger = 0; // round robin of the contact search
	bool m_solve_all = true;
};
