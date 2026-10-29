#pragma once

#include <memory>

class attachable_hud_item;
class CWeapon;
class player_hud;
class IKinematics;

namespace Bodycam
{
struct ArmPose;

// The arm controllers on the HUD rig: the STALKER 2 style controller and the
// Classic layers (regrip, recoil, handling) solve both arms once per frame.
class HudArms
{
public:
	explicit HudArms(player_hud& hud);
	~HudArms();

	// Resolves the arm bones of the loaded HUD models.
	void Reset();
	void UpdateAndApply(const ArmPose& pose, float dt);
	void OnWeaponChanged(const attachable_hud_item* item);
	void DumpAuthoredMotionProfile() const;
	// This frame's regrip rotation of the camera, in camera-local space. False
	// when regrip does not move the camera this frame.
	bool ClassicRegripCamera(Fmatrix& local_delta) const;
	// A shot (firing) or the end of firing. True when recoil took the shot.
	bool ClassicRecoilFire(bool firing, bool aiming, const CWeapon* weapon);

private:
	struct State;

	std::unique_ptr<State> m_state;
};

// Two-bone arm IK on a HUD model: moves the hand (and its fingers) by hand_delta so it ends at
// target_hand, bends the elbow toward pole_elbow; the shoulder stays. twist may be BI_NONE.
void SolveHudArm(IKinematics* K, u16 upperarm, u16 forearm, u16 twist, u16 hand, const Fvector& target_hand,
	const Fvector& pole_elbow, const Fmatrix& hand_delta);
} // namespace Bodycam
