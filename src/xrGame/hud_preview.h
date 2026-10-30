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
	bool HasBone(LPCSTR id, LPCSTR bone);
	// HUD-space matrix of a bone of a node model (identity if no bone); a hidden bone is placed by its bind pose
	// relative to the parent, so the point stays where the bone would be when shown
	Fmatrix BoneTransform(LPCSTR id, LPCSTR bone);
	void SetBoneVisible(LPCSTR id, LPCSTR bone, bool visible);
	void CopyBonesVisibility(LPCSTR id, CScriptGameObject* obj); // by bone names, from the object's HUD model (else world)
	bool Play(LPCSTR id, LPCSTR motion, bool mix);      // looped cycle on an animated model
	// bone poses (by name) from a model of the object: once - captured now and kept, else copied every frame;
	// nil obj - back to the model's own pose. part: "" - the object's HUD model (else world model),
	// "scope" - the MAS scope HUD model of a weapon (root at the mount, attach_scale)
	void FollowPose(LPCSTR id, CScriptGameObject* obj, bool once, LPCSTR part);
	void SetScale(LPCSTR id, float scale);              // uniform scale of the node's model (as attach_scale)
	void SetVisible(LPCSTR id, bool visible);          // false - not drawn in the HUD (capture only), children too
	Fvector BoxCenter(LPCSTR id);                       // bounding box of the node's model in its own space
	Fvector BoxSize(LPCSTR id);                         // (all parts, hidden ones too)
	// Render to texture: node id and its children into texture ("$user$name", for UI: CUIStatic:InitTexture),
	// right now. cam - camera in the node's space (c - position, k - view direction, j - up), orthographic,
	// width_m x height_m meters, px_per_m pixels per meter; transparent background. Returns pixel bounds of
	// what was drawn (x2 < x1 - nothing, or no DX11). Light: CaptureLight.
	Frect Capture(LPCSTR id, LPCSTR texture, const Fmatrix& cam, float width_m, float height_m, float px_per_m);
	void CaptureLight(const Fvector& to_light, float ambient); // view space direction to the key light; ambient 0..1
	void ReleaseTexture(LPCSTR texture);
	void HideHands(bool hide);
	bool HandsHidden();

	// engine side
	void Update(const Fmatrix& hud_camera);             // player_hud::update
	void Render(IDSGraphManager* DM);                   // player_hud::render_hud
}
