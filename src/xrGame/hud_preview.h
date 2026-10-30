#pragma once
// HUD preview: script-controlled 3D models rendered in the HUD pass (like the weapon in hands), optionally with
// the hands and hand items hidden. Nodes are models (or empty transform nodes) placed relative to the HUD camera,
// or attached to another node with a pos/rot offset. Lua: module hud_preview (player_hud_script.cpp).
#include "../Include/xrRender/KinematicsAnimated.h"

class IDSGraphManager;
class CScriptGameObject;

namespace HudPreview
{
	bool Add(LPCSTR id, LPCSTR visual);                 // visual "" -> empty node; replaces an existing id
	void Remove(LPCSTR id);                             // children are removed too
	void Clear();
	bool Exists(LPCSTR id);
	void SetTransform(LPCSTR id, const Fmatrix& m);     // relative to the HUD camera (x right, y up, z forward)
	void Attach(LPCSTR id, LPCSTR parent, const Fvector& pos, const Fvector& rot); // rot rad, X then Y then Z
	void Detach(LPCSTR id);
	Fmatrix Transform(LPCSTR id);                       // final HUD-space matrix (points -> game.world2ui(p, true))
	void SetBoneVisible(LPCSTR id, LPCSTR bone, bool visible);
	void CopyBonesVisibility(LPCSTR id, CScriptGameObject* obj); // by bone names, from the object's HUD model (else world)
	bool Play(LPCSTR id, LPCSTR motion, bool mix);      // looped cycle on an animated model
	void FollowPose(LPCSTR id, CScriptGameObject* obj); // every frame copy bone poses (by name) from the object's
	                                                    // HUD model (else world model); nil stops
	void HideHands(bool hide);
	bool HandsHidden();

	// engine side
	void Update(const Fmatrix& hud_camera);             // player_hud::update
	void Render(IDSGraphManager* DM);                   // player_hud::render_hud
}
