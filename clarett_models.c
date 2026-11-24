// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — the model table and model
 * detection.
 */
#include <linux/pci.h>
#include <linux/slab.h>
#include "clarett.h"

/* --- per-model descriptors ---------------------------------------------- */

/*
 * "Line NN (descr)" format matching the USB models; the 8PreX has no USB
 * sibling, so only the monitor pair carries a descr (outputs 3-10 are generic
 * line outs per its XML).
 */
static const struct clarett_out_gain clarett_8prex_gains[] = {
	{ "Line 01 (Monitor L)", 32 }, { "Line 02 (Monitor R)", 33 },
	{ "Line 03", 36 }, { "Line 04", 37 },
	{ "Line 05", 40 }, { "Line 06", 41 },
	{ "Line 07", 44 }, { "Line 08", 45 },
	{ "Line 09", 48 }, { "Line 10", 49 },
};

/*
 * Hardware-meter sources for the 8PreX (activate 8). Selecting one writes its
 * three per-band channel-index tables (@136/146/156) then SET_DATA{184}.
 */
static const struct clarett_meter_source clarett_8prex_meter_sources[] = {
	{ "Analogue", 1, {
		{  0,  1,  2,  3,  4,  5,  6,  7, 26, 27 },
		{  0,  1,  2,  3,  4,  5,  6,  7, 18, 19 },
		{  0,  1,  2,  3,  4,  5,  6,  7, 14, 15 } } },
	{ "S/PDIF", 2, {
		{  8,  9, 255, 255, 255, 255, 255, 255, 26, 27 },
		{  8,  9, 255, 255, 255, 255, 255, 255, 18, 19 },
		{  8,  9, 255, 255, 255, 255, 255, 255, 14, 15 } } },
	{ "ADAT 1", 4, {
		{ 10, 11, 12, 13, 14, 15, 16, 17, 26, 27 },
		{ 10, 11, 12, 13, 255, 255, 255, 255, 18, 19 },
		{ 10, 11, 255, 255, 255, 255, 255, 255, 14, 15 } } },
	{ "ADAT 2", 8, {
		{ 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 },
		{ 14, 15, 16, 17, 255, 255, 255, 255, 18, 19 },
		{ 12, 13, 255, 255, 255, 255, 255, 255, 14, 15 } } },
};

/*
 * Selectable clock sources for the "Clock Source" control, Internal first.
 * Values are the SET_CLOCK enums (clarett.h): Internal 24, S/PDIF 3, ADAT 0,
 * shared by the whole line. The 8PreX adds its second ADAT port (1) and its
 * wordclock input (2).
 */
static const struct clarett_clock_src clarett_clock_srcs[] = {
	{ "Internal", CLARETT_CLOCK_INTERNAL },
	{ "S/PDIF",   CLARETT_CLOCK_SPDIF },
	{ "ADAT",     CLARETT_CLOCK_ADAT },
};

static const struct clarett_clock_src clarett_8prex_clock_srcs[] = {
	{ "Internal",  CLARETT_CLOCK_INTERNAL },
	{ "S/PDIF",    CLARETT_CLOCK_SPDIF },
	{ "ADAT 1",    CLARETT_CLOCK_ADAT },
	{ "ADAT 2",    CLARETT_CLOCK_ADAT2 },
	{ "Wordclock", CLARETT_CLOCK_WORDCLOCK },
};

static const struct clarett_model clarett_8prex = {
	.name = "Clarett 8PreX",
	.slug = "clarett-8prex",
	.id_family = MODEL_ID_FAMILY_CLARETT,
	.id_model = 2,
	.out_gains = clarett_8prex_gains,
	.n_out_gains = ARRAY_SIZE(clarett_8prex_gains),
	.has_midi = true,
	.notify_word = true,
	.meter_sources = clarett_8prex_meter_sources,
	.n_meter_sources = ARRAY_SIZE(clarett_8prex_meter_sources),
	.capture_channels = STREAM_CHANS,
	.playback_channels = STREAM_CHANS,
	.rx_live_mid = 20,    /* two ADAT ports: ch20-27 gone at double speed */
	.clock_srcs = clarett_8prex_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(clarett_8prex_clock_srcs),
	.rx_live_high = 16,   /* + ch16-19 gone at quad */
	.loopback_last = true,
	.rx_loopback_at = 10, /* after analogue 1-8 + S/PDIF 1-2 */
};

/*
 * Clarett 2Pre (Thunderbolt): 4 playback / 14 record, the first 4 of the 8PreX
 * output gains. The names mirror scarlett2's Clarett 2Pre USB line_out_descrs,
 * in its "Line %02d (%s)" format.
 */
static const struct clarett_out_gain clarett_2pre_gains[] = {
	{ "Line 01 (Monitor L)", 32 }, { "Line 02 (Monitor R)", 33 },
	{ "Line 03 (Headphones L)", 36 }, { "Line 04 (Headphones R)", 37 },
};

const struct clarett_model clarett_2pre = {
	.name = "Clarett 2Pre",
	.slug = "clarett-2pre",
	.id_family = MODEL_ID_FAMILY_CLARETT,
	.id_model = 4,
	.out_gains = clarett_2pre_gains,
	.n_out_gains = ARRAY_SIZE(clarett_2pre_gains),
	.has_midi = true,
	.notify_word = true,
	/* record-outputs pin count (12 record + 2 loopback) */
	.capture_channels = 14,
	/* playback pin count */
	.playback_channels = 4,
	/* ADAT 5-8 -> ch10-13 gone at double speed */
	.rx_live_mid = 10,
	.clock_srcs = clarett_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(clarett_clock_srcs),
	/* + ADAT 3-4 -> ch8-9 gone at quad */
	.rx_live_high = 8,
	/* after analogue 1-2 + S/PDIF 1-2 */
	.loopback_last = true,
	.rx_loopback_at = 4,
};

/*
 * Clarett 4Pre (Thunderbolt): 8 playback / 20 record (18 record + 2 loopback),
 * six output gains (no 44/45 pair). Names as scarlett2's Clarett 4Pre USB.
 */
static const struct clarett_out_gain clarett_4pre_gains[] = {
	{ "Line 01 (Monitor L)", 32 }, { "Line 02 (Monitor R)", 33 },
	{ "Line 03 (Headphones 1 L)", 36 }, { "Line 04 (Headphones 1 R)", 37 },
	{ "Line 05 (Headphones 2 L)", 40 }, { "Line 06 (Headphones 2 R)", 41 },
};

static const struct clarett_model clarett_4pre = {
	.name = "Clarett 4Pre",
	.slug = "clarett-4pre",
	.id_family = MODEL_ID_FAMILY_CLARETT,
	.id_model = 3,
	.out_gains = clarett_4pre_gains,
	.n_out_gains = ARRAY_SIZE(clarett_4pre_gains),
	.has_midi = true,
	.notify_word = true,
	.capture_channels = 20,	/* record-outputs pin count */
	.playback_channels = 8,	/* playback pin count */
	.rx_live_mid = 16,	/* ADAT 5-8 -> ch16-19 gone at double speed */
	.clock_srcs = clarett_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(clarett_clock_srcs),
	.rx_live_high = 14,	/* + ADAT 3-4 -> ch14-15 gone at quad */
	.loopback_last = true,
	.rx_loopback_at = 10,	/* after analogue 1-8 + S/PDIF 1-2 */
};

/*
 * Clarett 8Pre (Thunderbolt), a different model from the 8PreX: one ADAT bank,
 * so 20 playback / 20 record against the 8PreX's 28/28. The 8PreX's ten gain
 * offsets, named as scarlett2's Clarett 8Pre USB (Monitor L/R, four unlabelled
 * line outs, Headphones 1/2).
 */
static const struct clarett_out_gain clarett_8pre_gains[] = {
	{ "Line 01 (Monitor L)", 32 }, { "Line 02 (Monitor R)", 33 },
	{ "Line 03", 36 }, { "Line 04", 37 },
	{ "Line 05", 40 }, { "Line 06", 41 },
	{ "Line 07 (Headphones 1 L)", 44 }, { "Line 08 (Headphones 1 R)", 45 },
	{ "Line 09 (Headphones 2 L)", 48 }, { "Line 10 (Headphones 2 R)", 49 },
};

static const struct clarett_model clarett_8pre = {
	.name = "Clarett 8Pre",
	.slug = "clarett-8pre",
	.id_family = MODEL_ID_FAMILY_CLARETT,
	.id_model = 1,
	.out_gains = clarett_8pre_gains,
	.n_out_gains = ARRAY_SIZE(clarett_8pre_gains),
	.has_midi = true,
	.notify_word = true,
	.capture_channels = 20,  /* 18 record + 2 loopback */
	.playback_channels = 20, /* Playback 1-20 */
	.rx_live_mid = 16,       /* ADAT 5-8 -> ch16-19 gone at double speed */
	.clock_srcs = clarett_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(clarett_clock_srcs),
	.rx_live_high = 14,      /* + ADAT 3-4 -> ch14-15 gone at quad */
	.loopback_last = true,
	.rx_loopback_at = 10,    /* after analogue 1-8 + S/PDIF 1-2 */
};

/*
 * Focusrite Red 8Line: playback=64, capture=60 (58 record + 2 loopback). Left
 * empty on purpose:
 *  - out_gains: the Red's output gains are 16-bit values on a 2-byte stride,
 *    and clarett_hw_gain_follow() writes a byte, so it would write half a gain
 *    field;
 *  - meter_sources: the Red's front-panel meter bridge is not mapped;
 *  - has_midi: the Red range has no MIDI.
 * Its controls are fcp-server's, through the FCP hwdep.
 */
/*
 * The same values on every Red model (the Pre models' descriptors list them in
 * another order).
 */
static const struct clarett_clock_src red_clock_srcs[] = {
	{ "Internal",  CLARETT_CLOCK_INTERNAL },
	{ "Wordclock", CLARETT_CLOCK_WORDCLOCK },
	{ "ADAT 1",    CLARETT_CLOCK_ADAT },
	{ "ADAT 2",    CLARETT_CLOCK_ADAT2 },
	{ "S/PDIF",    CLARETT_CLOCK_SPDIF },
	{ "Dante",     CLARETT_CLOCK_DANTE },
	{ "Loop Sync", CLARETT_CLOCK_LOOPSYNC },
};

static const struct clarett_model red_8line = {
	.name = "Red 8Line",
	.slug = "red-8line",
	.id_family = MODEL_ID_FAMILY_RED,
	.id_model = 6,
	.fw_front = true,		/* APPSPACE_FW_FRONT */
	.notify_word = true,		/* NOTIFY_EV_RED_* */
	.capture_channels = 60,			/* 58 record + 2 loopback */
	.playback_channels = 64,		/* Playback 1-64 */
	/*
	 * S/MUX: unlike the Claretts, the Red RE-PINS surviving channels (a
	 * slot carrying ADAT 5 at single speed carries ADAT 9 at double and
	 * Dante 1 at quad), but the channels that go are still a contiguous
	 * tail, which is what these two counts require.
	 */
	.rx_live_mid = 52,  /* ADAT 1-4 + 9-12, all 32 Dante: 8 slots fewer */
	.rx_live_high = 32, /* ADAT 1-2 + 9-10, Dante 1-16 */
	/*
	 * Playback drops its tail too (63-64 at double speed, 37-64 at quad);
	 * the device ignores those channels, so nothing needs blanking.
	 */
	.clock_srcs = red_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_clock_srcs),
	.tb_gen = 3,	/* JHL6540 Alpine Ridge; the Red 4Pre is its TB2 twin */
};

/*
 * Focusrite Red 16Line, NOT YET SEEN ON HARDWARE: taken from its device
 * descriptor. The Red 8Line plus eight line inputs and outputs, otherwise the
 * same. Geometry playback=64, capture=64 (62 record + 2 loopback).
 */
static const struct clarett_model red_16line = {
	.name = "Red 16Line",
	.slug = "red-16line",
	.fw_front = true,		/* APPSPACE_FW_FRONT */
	.notify_word = true,		/* NOTIFY_EV_RED_* */
	.capture_channels = 64,			/* 62 record + 2 loopback */
	.playback_channels = 64,		/* Playback 1-64 */
	/*
	 * The 8Line's S/MUX re-pinning, over a wider stream; both dead sets are
	 * contiguous tails.
	 */
	/* ADAT 1-4 + 9-12, all 32 Dante: Dante 25-28's slots go */
	.rx_live_mid = 60,
	/* ADAT 1-2 + 9-10, Dante 1-16 */
	.rx_live_high = 40,
	/*
	 * Playback 45-64 drop out at quad speed; the device ignores them, as on
	 * the 8Line.
	 */
	.clock_srcs = red_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_clock_srcs),
	/* its TB2 twin is the Red 8Pre */
	.tb_gen = 3,
};

/*
 * Focusrite Red 4Pre and Red 8Pre, NOT YET SEEN ON HARDWARE: taken from their
 * device descriptors. Each is its Line model's twin (the same stream geometry,
 * S/MUX tails, clock sources and config layout; the extra preamps are
 * fcp-server's), told apart by the unit's own Thunderbolt controller: the Pre
 * models are Thunderbolt 2 units, the Line models Thunderbolt 3.
 */
static const struct clarett_model red_4pre = {
	.name = "Red 4Pre",
	.slug = "red-4pre",
	.fw_front = true,	/* APPSPACE_FW_FRONT */
	.notify_word = true,	/* NOTIFY_EV_RED_* */
	.capture_channels = 60,	/* 58 record + 2 loopback, as the 8Line */
	.playback_channels = 64,
	.rx_live_mid = 52,
	.rx_live_high = 32,
	.clock_srcs = red_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_clock_srcs),
	.tb_gen = 2,
};

static const struct clarett_model red_8pre = {
	.name = "Red 8Pre",
	.slug = "red-8pre",
	.fw_front = true,	/* APPSPACE_FW_FRONT */
	.notify_word = true,	/* NOTIFY_EV_RED_* */
	.capture_channels = 64,	/* 62 record + 2 loopback, as the 16Line */
	.playback_channels = 64,
	.rx_live_mid = 60,
	.rx_live_high = 40,
	.clock_srcs = red_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_clock_srcs),
	.tb_gen = 2,
};

/*
 * Family defaults, for a unit no entry above claims: what every model of the
 * family shares (clock sources, notification bits, MIDI on a Clarett, the
 * front-panel firmware word on a Red), and nothing that is per model (output
 * gain offsets, the meter-source tables, the loopback position, which is left
 * in the device's stream order). The stream is sized from the device's own
 * geometry, so such a unit streams at full width. The geometry and S/MUX
 * fields stay 0: nothing matches against them and the device reports S/MUX.
 */
static const struct clarett_model clarett_family = {
	.name = "Clarett",
	.slug = "clarett-unknown",
	.has_midi = true,
	.loopback_last = true,
	.notify_word = true,
	.clock_srcs = clarett_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(clarett_clock_srcs),
};

static const struct clarett_model red_family = {
	.name = "Red",
	.slug = "red-unknown",
	.fw_front = true,
	.notify_word = true,
	.clock_srcs = red_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_clock_srcs),
};

/* Neither family: Internal clock and single speed only, the wildcard relay. */
static const struct clarett_clock_src internal_only[] = {
	{ "Internal", CLARETT_CLOCK_INTERNAL },
};

static const struct clarett_model unknown_family = {
	.name = "Thunderbolt Audio",
	.slug = "unknown",
	.clock_srcs = internal_only,
	.n_clock_srcs = ARRAY_SIZE(internal_only),
};

/*
 * Model selection: the device decides, and there is deliberately no override.
 * The whole line shares PCI id 1cb5:0002 and an identical pre-mailbox register
 * surface, and the Thunderbolt DROM's model name is not visible on every host.
 * STREAM_INFO{band 0} answers {u16 playback_ch, u16 capture_ch}: the readiness
 * test, and the stream geometry the rings and the PCM are sized from, whatever
 * the model. The model itself comes from MODEL_ID {family, model}, which is
 * what the vendor driver asks first, then from the geometry for a model whose
 * MODEL_ID has not been read from a unit (unique per model bar the Red twins).
 * A unit neither identifies gets its family's defaults and still attaches.
 */

/*
 * The Thunderbolt generation of the unit's OWN controller: 3, 2 (meaning any
 * pre-Thunderbolt 3 part), or 0 if it cannot be told.
 *
 * The endpoint's immediate upstream bridge is always the unit's own Thunderbolt
 * controller, whatever sits further up the chain (a dock, an adapter, another
 * unit). Every Clarett carries a DSL2210 Port Ridge and the Red 8Line a JHL6540
 * Alpine Ridge. The Red 4Pre and 8Pre are Thunderbolt 2 units, the 8Line and
 * 16Line Thunderbolt 3, so this tells apart the Red models that report the same
 * stream geometry.
 *
 * 0 covers no upstream bridge (the device passed through to a virtual machine)
 * and any controller not listed; clarett_detect_model() then takes the first
 * matching model.
 */
static int clarett_unit_tb_gen(struct pci_dev *pdev)
{
	static const u16 tb3[] = {
		0x1576, 0x1578,		/* Alpine Ridge 2C / 4C */
		0x15c0,			/* Alpine Ridge LP */
		0x15d3, 0x15da,		/* Alpine Ridge C step, 4C / 2C */
		0x15e7, 0x15ea, 0x15ef,	/* Titan Ridge 2C / 4C / DD */
	};
	static const u16 tb2[] = {
		0x1513, 0x151a,		/* Light Ridge, Eagle Ridge */
		0x1547, 0x1548, 0x1549,	/* Cactus Ridge 4C / 2C, Port Ridge */
		0x1567, 0x1569,		/* Redwood Ridge 2C / 4C */
		0x156b, 0x156d,		/* Falcon Ridge 2C / 4C */
	};
	struct pci_dev *br = pci_upstream_bridge(pdev);
	int i;

	if (!br || br->vendor != PCI_VENDOR_ID_INTEL)
		return 0;
	for (i = 0; i < ARRAY_SIZE(tb3); i++)
		if (br->device == tb3[i])
			return 3;
	for (i = 0; i < ARRAY_SIZE(tb2); i++)
		if (br->device == tb2[i])
			return 2;
	return 0;
}

/*
 * STREAM_INFO{band}: the device's stream geometry at one speed band. Returns 0
 * with the pair filled in, -EIO if the device refused the read (status != 0, or
 * a reply too short to hold the pair), or the transport error. The reply is
 * copied out under mbox_lock, so another command (a meter read, say) cannot
 * overwrite it mid-parse. When not quiet, every failure logs the raw response.
 */
static int clarett_stream_info(struct clarett *c, u8 band, u16 *playback,
			       u16 *capture, bool quiet)
{
	u8 r[4];
	u16 size;
	u32 status;
	int err;

	err = clarett_fcp_read(c, FCP_STREAM_INFO, &band, 1, r, sizeof(r),
			       &size, &status);
	if (err) {
		if (!quiet)
			dev_warn(&c->pci->dev,
				 "STREAM_INFO{%u}: transport failed (%d)\n",
				 band, err);
		return err;
	}
	*playback = clarett_get_le16(r);
	*capture  = clarett_get_le16(r + 2);
	if (status != FCP_RESP_ERR_OK || size < 4) {
		if (!quiet)
			dev_warn(&c->pci->dev,
				 "STREAM_INFO{%u}: bad response (status=%u size=%u raw playback=%u capture=%u)\n",
				 band, status, size, *playback, *capture);
		return -EIO;
	}
	return 0;
}

static const struct clarett_model *const clarett_models[] = {
	&clarett_2pre, &clarett_4pre, &clarett_8pre, &clarett_8prex,
	&red_8line, &red_16line, &red_4pre, &red_8pre,
};

/*
 * MODEL_ID {0, 8}: the device's {family, model} into c->id_family/id_model.
 * Sets c->id_ok only on a full answer; any failure leaves the geometry to
 * decide, so a firmware that refuses the query still attaches as before.
 */
static void clarett_read_model_id(struct clarett *c)
{
	u8 req[8], r[8];
	u16 size;
	u32 status;

	c->id_ok = false;
	clarett_put_le32(req, 0);
	clarett_put_le32(req + 4, sizeof(r));
	if (clarett_fcp_read(c, FCP_MODEL_ID, req, sizeof(req), r, sizeof(r),
			     &size, &status) ||
	    status != FCP_RESP_ERR_OK || size < sizeof(r))
		return;
	c->id_family = clarett_get_le32(r);
	c->id_model = clarett_get_le32(r + 4);
	c->id_ok = true;
}

/*
 * Match the stream geometry, among the models whose MODEL_ID is unknown when
 * the device answered one no entry claims (a measured model is never chosen by
 * geometry against its own ID), or among all of them when it gave none. The
 * Red twins (4Pre/8Line, 8Pre/16Line) share a pair and carry tb_gen: the unit's
 * own Thunderbolt controller decides, and if it cannot be told the first
 * candidate is taken (clarett_log_ident() says so).
 */
static const struct clarett_model *
clarett_match_geometry(struct clarett *c, u16 pb, u16 cap)
{
	const struct clarett_model *first = NULL;
	int gen = clarett_unit_tb_gen(c->pci);
	int i;

	for (i = 0; i < ARRAY_SIZE(clarett_models); i++) {
		const struct clarett_model *m = clarett_models[i];

		if (c->id_ok && m->id_family)
			continue;
		if (m->playback_channels != pb || m->capture_channels != cap)
			continue;
		if (!first)
			first = m;
		if (gen && m->tb_gen == gen)
			return m;
	}
	return first;
}

/* The family of a unit no entry claims: MODEL_ID, else register 0x000. */
static const struct clarett_model *clarett_family_model(struct clarett *c)
{
	u32 family = c->id_family;

	if (!c->id_ok) {
		/* the FPGA word: 0x0320 Clarett, 0x0420 Red, in bits 31:16 */
		switch (clarett_rl(c, REG_CAPS) >> 16) {
		case 0x0320:
			family = MODEL_ID_FAMILY_CLARETT;
			break;
		case 0x0420:
			family = MODEL_ID_FAMILY_RED;
			break;
		default:
			family = 0;
		}
	}
	if (family == MODEL_ID_FAMILY_CLARETT)
		return &clarett_family;
	if (family == MODEL_ID_FAMILY_RED)
		return &red_family;
	return &unknown_family;
}

/* The table entry MODEL_ID names, whatever its geometry, or NULL. */
static const struct clarett_model *clarett_model_by_id(struct clarett *c)
{
	int i;

	for (i = 0; c->id_ok && i < ARRAY_SIZE(clarett_models); i++)
		if (clarett_models[i]->id_family == c->id_family &&
		    clarett_models[i]->id_model == c->id_model)
			return clarett_models[i];
	return NULL;
}

/*
 * Ask the device who it is: STREAM_INFO{band 0}, which also sets the stream
 * geometry (c->playback_ch, c->capture_ch), then MODEL_ID. Returns the model:
 * the entry MODEL_ID names if its geometry agrees, else the entry the geometry
 * matches, else the unit's family defaults. Returns NULL only when the device
 * gave no usable geometry, which is fatal to probe; *collapsed says why: true =
 * the read failed or was refused (usually "not ready yet, retry"), false = a
 * width outside 1..CLARETT_MAX_STREAM_CHANS.
 */
const struct clarett_model *
clarett_detect_model(struct clarett *c, bool *collapsed, bool quiet)
{
	const struct clarett_model *m;
	u16 pb, cap;

	/* when not quiet, every failure logs the raw response */
	*collapsed = clarett_stream_info(c, 0, &pb, &cap, quiet) != 0;
	if (*collapsed)
		return NULL;
	if (!pb || !cap || pb > CLARETT_MAX_STREAM_CHANS ||
	    cap > CLARETT_MAX_STREAM_CHANS) {
		if (!quiet)
			dev_warn(&c->pci->dev,
				 "STREAM_INFO{0}: stream geometry playback=%u capture=%u outside 1..%u\n",
				 pb, cap, CLARETT_MAX_STREAM_CHANS);
		return NULL;
	}
	c->playback_ch = pb;
	c->capture_ch = cap;

	clarett_read_model_id(c);
	m = clarett_model_by_id(c);
	if (m) {
		if (m->playback_channels == pb && m->capture_channels == cap)
			return m;
		return clarett_family_model(c);
	}

	return clarett_match_geometry(c, pb, cap) ?: clarett_family_model(c);
}

/*
 * Say how the model was identified, in one line unless MODEL_ID named it: a
 * model chosen by geometry gets the MODEL_ID answer to add to its entry, a unit
 * on its family's defaults says why, and a Red twin names the controller and
 * the FPGA word (register 0x000) that confirm or correct the rule.
 */
void clarett_log_ident(struct clarett *c)
{
	const struct clarett_model *m = c->model;
	const struct clarett_model *named = clarett_model_by_id(c);
	struct pci_dev *br = pci_upstream_bridge(c->pci);
	int gen = clarett_unit_tb_gen(c->pci);

	if (!m->playback_channels) {
		/* family defaults: no entry claimed the unit */
		if (named)
			dev_warn(&c->pci->dev,
				 "MODEL_ID {%u, %u} names the %s, but the device reports playback=%u capture=%u against the table's %u/%u: using the %s family defaults\n",
				 c->id_family, c->id_model, named->name,
				 c->playback_ch, c->capture_ch,
				 named->playback_channels,
				 named->capture_channels, m->name);
		else if (c->id_ok)
			dev_warn(&c->pci->dev,
				 "unknown model (MODEL_ID {%u, %u}, playback=%u capture=%u, 0x000 = 0x%08x): using the %s family defaults; it needs a clarett_model entry\n",
				 c->id_family, c->id_model, c->playback_ch,
				 c->capture_ch, clarett_rl(c, REG_CAPS),
				 m->name);
		else
			dev_warn(&c->pci->dev,
				 "unknown model (MODEL_ID unanswered, playback=%u capture=%u, 0x000 = 0x%08x): using the %s family defaults; it needs a clarett_model entry\n",
				 c->playback_ch, c->capture_ch,
				 clarett_rl(c, REG_CAPS), m->name);
		return;
	}
	if (named == m) {
		dev_dbg(&c->pci->dev, "%s: identified by MODEL_ID {%u, %u}\n",
			m->name, c->id_family, c->id_model);
		return;
	}
	if (c->id_ok)
		dev_info(&c->pci->dev,
			 "%s: chosen by stream geometry; its MODEL_ID is {%u, %u}, which the model table does not list yet\n",
			 m->name, c->id_family, c->id_model);
	else
		dev_warn(&c->pci->dev,
			 "%s: chosen by stream geometry, as the device did not answer MODEL_ID\n",
			 m->name);
	if (!m->tb_gen)
		return;
	if (gen == m->tb_gen)
		dev_info(&c->pci->dev,
			 "%s: identified by its Thunderbolt %d controller %04x:%04x (0x000 = 0x%08x)\n",
			 m->name, gen, br->vendor, br->device,
			 clarett_rl(c, REG_CAPS));
	else
		dev_warn(&c->pci->dev,
			 "%s assumed: its geometry twin cannot be ruled out, as this unit's Thunderbolt controller (%04x:%04x) is not recognised (0x000 = 0x%08x)\n",
			 m->name, br ? br->vendor : 0, br ? br->device : 0,
			 clarett_rl(c, REG_CAPS));
}

/*
 * Read which speed bands the device runs and how many leading capture channels
 * it writes in each: STREAM_INFO's capture word at bands 1 and 2 (band 0 is the
 * full width by definition), into c->rx_live[] and c->speed_bands. A band the
 * device does not answer, or answers with a width outside the frame, ends the
 * list: its rates and those above are not offered. The model table
 * (rx_live_mid/high) is a cross-check only: a disagreement is logged, and so is
 * a missing band the table says the model runs. Run after the model is final,
 * since the Red twins share band 0 but not necessarily the table.
 */
void clarett_read_rx_live(struct clarett *c)
{
	const struct clarett_model *m = c->model;
	u8 width = c->capture_ch;
	unsigned int band;

	/* a band not offered is never used; full width keeps it harmless */
	for (band = 0; band < CLARETT_SPEED_BANDS; band++)
		c->rx_live[band] = width;
	for (band = 1; band < CLARETT_SPEED_BANDS; band++) {
		u8 table = band == 1 ? m->rx_live_mid : m->rx_live_high;
		u16 pb, cap;
		int err;

		err = clarett_stream_info(c, band, &pb, &cap, !table);
		if (!err && (!cap || cap > width)) {
			if (table)
				dev_warn(&c->pci->dev,
					 "STREAM_INFO{%u}: capture width %u outside 1..%u\n",
					 band, cap, width);
			err = -ERANGE;
		}
		if (err) {
			if (table)
				dev_warn(&c->pci->dev,
					 "STREAM_INFO{%u} unusable: the %s should run at this speed; offering only the rates below it\n",
					 band, m->name);
			break;
		}
		if (table && cap != table)
			dev_warn(&c->pci->dev,
				 "STREAM_INFO{%u}: device writes %u capture channels, the %s table says %u; using the device's\n",
				 band, cap, m->name, table);
		c->rx_live[band] = cap;
	}
	c->speed_bands = band;
	dev_dbg(&c->pci->dev,
		"speed bands %u, capture channels live per band: %u/%u/%u\n",
		c->speed_bands, c->rx_live[0], c->rx_live[1], c->rx_live[2]);
}

/* PORT_NAME{id} into buf, printable ASCII only; false if refused or empty. */
static bool clarett_port_name(struct clarett *c, u16 id, char *buf,
			      size_t len)
{
	u8 req[2], r[CLARETT_CLOCK_NAME_LEN];
	u16 size;
	u32 status;
	size_t i;

	clarett_put_le16(req, id);
	if (clarett_fcp_read(c, FCP_PORT_NAME, req, sizeof(req), r, sizeof(r),
			     &size, &status) || status != FCP_RESP_ERR_OK)
		return false;
	size = min_t(size_t, size, min(sizeof(r), len - 1));
	for (i = 0; i < size && r[i]; i++) {
		if (r[i] < 0x20 || r[i] > 0x7e)
			return false;
		buf[i] = r[i];
	}
	buf[i] = '\0';
	return i > 0;
}

/* The model's table as the Clock Source list, when the device gave none. */
static void clarett_table_clock_srcs(struct clarett *c)
{
	const struct clarett_model *m = c->model;
	unsigned int i;

	c->n_clock_srcs = min_t(unsigned int, m->n_clock_srcs,
				CLARETT_MAX_CLOCK_SRCS);
	for (i = 0; i < c->n_clock_srcs; i++)
		c->clock_srcs[i] = m->clock_srcs[i];
}

/*
 * Build the "Clock Source" items from the device: CLOCK_LIST gives each
 * source's SET_CLOCK value and name id, PORT_NAME the name. This is what the
 * vendor driver does, and it is where the 2Pre's S/PDIF value (4, not the
 * other Claretts' 3) comes from. A name the device will not give is taken from
 * the model's table for that value, else made up from the value; a list the
 * device will not give falls back to the model's table.
 */
void clarett_read_clock_srcs(struct clarett *c)
{
	const struct clarett_model *m = c->model;
	u8 r[4 * CLARETT_MAX_CLOCK_SRCS];
	unsigned int i, j, n;
	u16 size;
	u32 status;

	c->n_clock_srcs = 0;
	if (clarett_fcp_read(c, FCP_CLOCK_LIST, NULL, 0, r, sizeof(r), &size,
			     &status) || status != FCP_RESP_ERR_OK) {
		dev_warn(&c->pci->dev,
			 "CLOCK_LIST unanswered: using the %s table's clock sources\n",
			 m->name);
		clarett_table_clock_srcs(c);
		return;
	}

	n = min_t(unsigned int, size, sizeof(r)) / 4;
	for (i = 0; i < n; i++) {
		struct clarett_clock_src *src = &c->clock_srcs[c->n_clock_srcs];
		char *name = c->clock_names[c->n_clock_srcs];
		u16 value = clarett_get_le16(r + 4 * i);
		u16 id = clarett_get_le16(r + 4 * i + 2);

		/* name id 0 is the empty string: the end of the list */
		if (!id)
			break;
		if (value > U8_MAX)
			continue;
		if (!clarett_port_name(c, id, name, CLARETT_CLOCK_NAME_LEN)) {
			snprintf(name, CLARETT_CLOCK_NAME_LEN, "Source %u",
				 value);
			for (j = 0; j < m->n_clock_srcs; j++)
				if (m->clock_srcs[j].value == value)
					strscpy(name, m->clock_srcs[j].name,
						CLARETT_CLOCK_NAME_LEN);
		}
		src->name = name;
		src->value = value;
		c->n_clock_srcs++;
	}

	if (!c->n_clock_srcs) {
		dev_warn(&c->pci->dev,
			 "CLOCK_LIST empty: using the %s table's clock sources\n",
			 m->name);
		clarett_table_clock_srcs(c);
		return;
	}
	for (i = 0; i < c->n_clock_srcs; i++)
		dev_dbg(&c->pci->dev, "clock source %u: %s = %u\n", i,
			c->clock_srcs[i].name, c->clock_srcs[i].value);
}

/*
 * Where the capture stream's loopback pair is: STREAM_CHANS_RX{band 0} lists
 * every capture channel with its port class, and the pair is the first two
 * adjacent LOOP channels. Returns the first one's index, 0 if there is no such
 * pair, or a negative error if the device gave no usable list.
 */
static int clarett_find_loopback(struct clarett *c)
{
	size_t len = (size_t)c->capture_ch * CLARETT_CHAN_REC_BYTES;
	u8 req[6] = { 0 };
	unsigned int i, n;
	u16 size;
	u32 status;
	u8 *r;
	int err;

	r = kmalloc(len, GFP_KERNEL);
	if (!r)
		return -ENOMEM;
	/* band 0 (u32), then a count the device ignores */
	clarett_put_le16(req + 4, c->capture_ch);
	err = clarett_fcp_read(c, FCP_STREAM_CHANS_RX, req, sizeof(req), r,
			       len, &size, &status);
	if (!err && (status != FCP_RESP_ERR_OK || !size))
		err = -EIO;
	if (err)
		goto out;

	n = min_t(size_t, size, len) / CLARETT_CHAN_REC_BYTES;
	err = 0;
	for (i = 0; i + 1 < n; i++) {
		u8 *a = r + i * CLARETT_CHAN_REC_BYTES;
		u8 *b = a + CLARETT_CHAN_REC_BYTES;

		if (clarett_get_le16(a + 6) == CLARETT_PORT_CLASS_LOOP &&
		    clarett_get_le16(b + 6) == CLARETT_PORT_CLASS_LOOP) {
			err = i;
			break;
		}
	}
out:
	kfree(r);
	return err;
}

/*
 * Set c->rx_loopback_at for a model that moves the loopback pair to the end of
 * each capture frame (loopback_last): from the device's channel list, else the
 * model's measured position. The pair must be live at every offered speed,
 * since the reorder assumes S/MUX removes only channels after it; otherwise
 * the stream is left in the device's order. Run after clarett_read_rx_live().
 */
void clarett_read_rx_layout(struct clarett *c)
{
	const struct clarett_model *m = c->model;
	u8 live = c->rx_live[c->speed_bands - 1];
	int at;

	c->rx_loopback_at = 0;
	if (!m->loopback_last)
		return;

	at = clarett_find_loopback(c);
	if (at < 0) {
		dev_warn(&c->pci->dev,
			 "capture channel list unanswered (%d): using the %s table's loopback position %u\n",
			 at, m->name, m->rx_loopback_at);
		at = m->rx_loopback_at;
	} else if (m->rx_loopback_at && at != m->rx_loopback_at) {
		dev_warn(&c->pci->dev,
			 "loopback pair at capture channel %d, the %s table says %u; using the device's\n",
			 at, m->name, m->rx_loopback_at);
	}
	if (at <= 0 || at + 2 > live) {
		if (at)
			dev_warn(&c->pci->dev,
				 "loopback pair at capture channel %d is not live at every speed (%u live); leaving the capture order alone\n",
				 at, live);
		return;
	}
	c->rx_loopback_at = at;
	dev_dbg(&c->pci->dev, "loopback pair at capture channel %d\n", at);
}

/*
 * Whether to create the MIDI device: from MIDI_INFO, the device's own count of
 * its MIDI ports (any port at all), else the model's table. A disagreement
 * with the table is reported and the device believed.
 */
void clarett_read_midi_ports(struct clarett *c)
{
	const struct clarett_model *m = c->model;
	u8 r[4];
	u16 size;
	u32 status;
	u16 a, b;

	if (clarett_fcp_read(c, FCP_MIDI_INFO, NULL, 0, r, sizeof(r), &size,
			     &status) || status != FCP_RESP_ERR_OK ||
	    size < sizeof(r)) {
		dev_warn(&c->pci->dev,
			 "MIDI_INFO unanswered: using the %s table (%s)\n",
			 m->name, m->has_midi ? "MIDI" : "no MIDI");
		c->has_midi = m->has_midi;
		return;
	}

	a = clarett_get_le16(r);
	b = clarett_get_le16(r + 2);
	c->has_midi = a || b;
	if (c->has_midi != m->has_midi)
		dev_warn(&c->pci->dev,
			 "MIDI_INFO reports %u/%u MIDI ports, the %s table says %s; using the device's\n",
			 a, b, m->name, m->has_midi ? "MIDI" : "no MIDI");
	dev_dbg(&c->pci->dev, "MIDI_INFO: %u/%u MIDI ports\n", a, b);
}
