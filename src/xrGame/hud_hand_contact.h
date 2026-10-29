#pragma once

class player_hud;
class IKinematics;

// Left hand vs the weapon for script hand motion (game.hud_hand_contact): when it turns on, the left
// hand is checked once against the weapon mesh; the palm is moved out of the weapon (arm IK) and
// fingers that go into it are opened just enough. The correction is fixed in the weapon frame and only
// faded in and out, so the hand doesn't move otherwise. Native weapon animations are left alone.
class HudHandContact
{
public:
	// weight 0..1 reached over fade seconds.
	void Request(float weight, float fade);
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
	};
	enum
	{
		kFingers = 5,
		kJoints = 3
	};

	bool ResolveBones(IKinematics* hand);
	void BuildGeometry(IKinematics* weapon);
	bool Closest(const Fvector& p, float range, Hit& hit) const;
	bool Inside(const Fvector& p, float radius) const; // item space
	bool FingerAxis(IKinematics* K, int finger, const Fmatrix& to_item, Fvector& axis) const;
	void Solve(IKinematics* K, const Fmatrix& to_item);

	float m_weight = 0.f, m_target = 0.f, m_speed = 0.f;
	bool m_solve = false;

	IKinematics* m_hand_model = nullptr;
	u16 m_upperarm = u16(-1), m_forearm = u16(-1), m_twist = u16(-1), m_hand = u16(-1);
	u16 m_finger[kFingers][kJoints];
	xr_vector<u16> m_moves[kFingers][kJoints]; // the joint and every bone below it

	IKinematics* m_weapon_model = nullptr;
	xr_vector<Tri> m_tris; // weapon model space
	xr_vector<xr_vector<u32>> m_cells;
	Fvector m_min;
	int m_nx = 0, m_ny = 0, m_nz = 0;
	mutable xr_vector<u32> m_stamp;
	mutable u32 m_visit = 0;

	// The correction found when contact turned on.
	Fvector m_push = {0.f, 0.f, 0.f};       // hand, left model space
	float m_open[kFingers][kJoints] = {};   // finger joints, radians (negative opens)
};
