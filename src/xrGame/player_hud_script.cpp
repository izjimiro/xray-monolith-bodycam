#include "pch_script.h"
#include "player_hud.h"
#include "hud_preview.h"
#include "script_game_object.h"

using namespace luabind;

player_hud* get_player_hud()
{
	return g_player_hud;
}

#pragma optimize("s",on)
void player_hud::script_register(lua_State* L)
{
	module(L)
	[
		class_<player_hud>("player_hud")
		.def(constructor<>())
		.def("set_hands", &player_hud::load_script)
		.def("reset_hands", &player_hud::reset_model_script),

		def("get_player_hud", &get_player_hud)
	];

	module(L, "hud_preview")
	[
		def("add", &HudPreview::Add),
		def("remove", &HudPreview::Remove),
		def("clear", &HudPreview::Clear),
		def("exists", &HudPreview::Exists),
		def("set_transform", &HudPreview::SetTransform),
		def("attach", &HudPreview::Attach),
		def("detach", &HudPreview::Detach),
		def("transform", &HudPreview::Transform),
		def("has_bone", &HudPreview::HasBone),
		def("bone_transform", &HudPreview::BoneTransform),
		def("set_bone_visible", &HudPreview::SetBoneVisible),
		def("copy_bones_visibility", &HudPreview::CopyBonesVisibility),
		def("play", &HudPreview::Play),
		def("follow_pose", &HudPreview::FollowPose),
		def("set_visible", &HudPreview::SetVisible),
		def("box_center", &HudPreview::BoxCenter),
		def("box_size", &HudPreview::BoxSize),
		def("capture", &HudPreview::Capture),
		def("capture_light", &HudPreview::CaptureLight),
		def("release_texture", &HudPreview::ReleaseTexture),
		def("hide_hands", &HudPreview::HideHands),
		def("hands_hidden", &HudPreview::HandsHidden)
	];
}
