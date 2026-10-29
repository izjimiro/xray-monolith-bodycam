#pragma once

class player_hud;

// Keyframed additive HUD pose clips fed from Lua (game.hud_clip_*). Tracks:
//   gun        both arms and the weapon, rigid, in the weapon frame (pivot lead_gun)
//   l_hand_ik  left hand target in the weapon frame, reached with arm IK
//   contact    0..1 left hand collision with the weapon (HudHandContact)
//   grasp      0..1 fingers wrap the weapon while in contact
//   <bone>     additive local offset of any hands bone and everything below it
// Transform keys: position in metres, rotation in degrees (x pitch, y yaw, z roll).
// Every track is relative to the pose the weapon animation gives, so one clip fits any gun.
class HudPoseClips
{
public:
	~HudPoseClips() { Reset(); }
	void Key(LPCSTR clip, LPCSTR track, float time, const Fvector& pos, const Fvector& rot);
	void Value(LPCSTR clip, LPCSTR track, float time, float value);
	void RemoveKey(LPCSTR clip, LPCSTR track, float time);
	void Remove(LPCSTR clip);
	void Play(LPCSTR clip, float speed, bool loop, float fade);
	void Stop(LPCSTR clip, float fade);
	void Pause(LPCSTR clip, bool paused);
	void SetTime(LPCSTR clip, float time);
	float Time(LPCSTR clip) const;
	float Length(LPCSTR clip) const;
	bool Playing(LPCSTR clip) const;

	// Once per frame in the HUD pose, after the script layers and before the items attach.
	void Apply(player_hud& hud);
	void Reset();

private:
	struct Key_
	{
		float t;
		Fvector pos, rot;
		float value;
	};
	struct Track
	{
		shared_str name;
		xr_vector<Key_> keys; // sorted by time
	};
	struct Clip
	{
		shared_str name;
		xr_vector<Track> tracks;
		float time = 0.f, speed = 1.f, weight = 0.f, target = 0.f, fade_speed = 5.f;
		bool loop = false, paused = false, active = false;
		float Length() const;
	};

	Clip* Find(LPCSTR name) const;
	Clip& Get(LPCSTR name);
	static Track& GetTrack(Clip& clip, LPCSTR name);
	static void Insert(Track& track, const Key_& key);
	static Key_ Sample(const Track& track, float t);

	xr_vector<Clip*> m_clips;
};
