/* B.Schaffl
 * MIDI Pattern Delay Plugin
 *
 * Copyright (C) 2018 - 2020 by Sven Jähnichen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef BSCHAFFL_H_
#define BSCHAFFL_H_

#define MODFL(x) (x - floorf (x))

#include <cmath>
#include <array>
#include <random>
#include <lv2/lv2plug.in/ns/ext/atom/atom.h>
#include <lv2/lv2plug.in/ns/ext/atom/forge.h>
#include <lv2/lv2plug.in/ns/ext/state/state.h>
#include "definitions.hpp"
#include "Urids.hpp"
#include "Ports.hpp"
#include "Message.hpp"
#include "Limit.hpp"
#include "StaticArrayList.hpp"
#include "Shape.hpp"

struct MidiData
{
	uint8_t msg[3];
	size_t size;
	double positionSeq;
	double shiftSeq;
	double amp;
	bool inactive;
};

struct Atom_Ptr
{
	LV2_Atom atom;
	int nr;
	void* ptr;
};

const Limit controllerLimits [NR_CONTROLLERS] =
{
	{0.125, 16.0, 0},	// SEQ_LEN_VALUE
	{0, 2, 1},		// SEQ_LEN_BASE
	{0.0078125, 128.0, 0.0},// AMP_SWING
	{0.0, 1.0, 0.0},	// AMP_RANDOM
	{-1.0, 2.0, 0.0},	// AMP_PROCESS
	{0.333333, 3.0, 0.0},	// SWING
	{0.0, 1.0, 0.0},	// SWING_RANDOM
	{0.0, 1.0, 0.0},	// SWING_PROCESS
	{1, 16, 1},		// NR_OF_STEPS
	{0, 1, 1},		// AMP_MODE
	{0, 1, 1},		// MIDI_CH_FILTER
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},
	{0, 1, 1},		// MSG_FILTER_NOTE
	{0, 1, 1},		// MSG_FILTER_KEYPR
	{0, 1, 1},		// MSG_FILTER_CC
	{0, 1, 1},		// MSG_FILTER_PROG
	{0, 1, 1},		// MSG_FILTER_CHPR
	{0, 1, 1},		// MSG_FILTER_PITCH
	{0, 1, 1},		// MSG_FILTER_SYS
	{0, 1, 1},		// NOTE_POSITION_STR
	{0, 1, 1},		// NOTE_VALUE_STR
	{0, 2, 1},		// NOTE_OVERLAP
	{0, 1, 1},		// NOTE_OFF_AMP
	{0.0, 0.5, 0},		// QUANT_RANGE
	{0, 1, 1},		// QUANT_MAP
	{0, 1, 1},		// QUANT_POS
	{0, 1, 1},		// TIME_COMPENS
	{0, 1, 1},		// USR_LATENCY
	{0, 192000, 1},		// USR_LATENCY_FR
	{0.0, 1.0, 0.0},	// STEP_POS
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},	// STEP_LEV
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 1.0, 0.0},
	{0, 192000, 1}		// LATENCY
};

class BSchaffl
{
public:
	BSchaffl (double samplerate, const LV2_Feature* const* features);
	~BSchaffl ();

	void connect_port (uint32_t port, void *data);
	void run (uint32_t n_samples);
	LV2_State_Status state_save(LV2_State_Store_Function store, LV2_State_Handle handle, uint32_t flags, const LV2_Feature* const* features);
	LV2_State_Status state_restore(LV2_State_Retrieve_Function retrieve, LV2_State_Handle handle, uint32_t flags, const LV2_Feature* const* features);

	LV2_URID_Map* map;

private:
	double rate;
	float bpm;
	float speed;
	int64_t bar;
	float barBeat;
	float beatsPerBar;
	int beatUnit;
	double positionSeq;
	double latencySeq;
	int64_t latencyFr;
	uint32_t refFrame;
	bool uiOn;
	int actStep;
	std::minstd_rand mcg;
	std::uniform_real_distribution<float> distribution;

	StaticArrayList<MidiData, MIDIBUFFERSIZE> midiData;

	// Data ports
	LV2_Atom_Sequence* input;
	LV2_Atom_Sequence* output;

	// Controllers
	int sharedDataNr;
	float* controllerPtrs[NR_CONTROLLERS];
	float controllers[NR_CONTROLLERS];
	float stepPositions[MAXSTEPS - 1];
	bool stepAutoPositions[MAXSTEPS - 1];
	float stepRnds[MAXSTEPS - 1];
	Shape<MAXNODES> shape;

	BSchafflURIs uris;

	LV2_Atom_Forge forge;
	LV2_Atom_Forge_Frame frame;

	Message message;
	bool notify_shape;
	bool notify_sharedData;
	bool notify_controllers[NR_CONTROLLERS];

	struct Atom_Controllers
	{
		LV2_Atom_Vector_Body body;
		float data[NR_CONTROLLERS];
	};

	float getControllerInput (const int sdNr, const int ctrlNr);
	void setController (const int ctrlNr, const float value);
	void randomizeStep (const int step);
	double getStepStart (const int step);
	double getStepEnd (const int step);
	int getNoteOnMsg (const uint8_t ch, const uint8_t note, int start = -1) const;
	int getNoteOffMsg (const uint8_t ch, const uint8_t note, int start = 0) const;
	void clearMidiData (const float maxSeq);
	void queueMidiData (const MidiData& midi);
	double getSequenceFromBeats (const double beats);
	double getBeatsFromSequence (const double sequence);
	double getSequenceFromFrame (const int64_t frames, float speed = 1.0f);
	int64_t getFrameFromSequence (const double sequence, float speed = 1.0f);
	bool filterMsg (const uint8_t msg);
	void recalculateLatency();
	void recalculateAutoPositions ();
	void play (uint32_t start, uint32_t end);
	void notifyControllerToGui (const int nr);
	void notifySharedDataNrToGui ();
	void notifyStatusToGui ();
	void notifyShapeToGui ();
	void notifyMessageToGui ();

};

#endif /* BSCHAFFL_H_ */
