// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — the model table and model
 * detection.
 */
#include <linux/pci.h>
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
	.rx_loopback_at = 10, /* after analogue 1-8 + S/PDIF 1-2 */
	.max_rate = 192000,
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
	.rx_loopback_at = 4,
	.max_rate = 192000,
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
	.rx_loopback_at = 10,	/* after analogue 1-8 + S/PDIF 1-2 */
	.max_rate = 192000,
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
	.rx_loopback_at = 10,    /* after analogue 1-8 + S/PDIF 1-2 */
	.max_rate = 192000,
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
	.max_rate = 192000,
	.clock_srcs = red_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_clock_srcs),
	.tb_gen = 3,	/* JHL6540 Alpine Ridge; the Red 4Pre is its TB2 twin */
	/*
	 * The three knob groups keep their gains at 112/116/120 and mute/dim at
	 * 124/126/128, and the Meter Source selector is at 268, so the watched
	 * region runs to 268 (the Clarett default stops at 115).
	 */
	.monitor_cfg_len = 269 - MONITOR_CFG_OFFSET,
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
	.max_rate = 192000,
	.clock_srcs = red_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_clock_srcs),
	/* the 8Line's knob groups and Meter Source */
	.monitor_cfg_len = 269 - MONITOR_CFG_OFFSET,
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
	.max_rate = 192000,
	.clock_srcs = red_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_clock_srcs),
	.monitor_cfg_len = 269 - MONITOR_CFG_OFFSET,
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
	.max_rate = 192000,
	.clock_srcs = red_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_clock_srcs),
	.monitor_cfg_len = 269 - MONITOR_CFG_OFFSET,
	.tb_gen = 2,
};

/*
 * Model selection: the device decides, and there is deliberately no override.
 * The whole line shares PCI id 1cb5:0002 and an identical pre-mailbox register
 * surface, and the Thunderbolt DROM's model name is not visible on every host.
 * But STREAM_INFO{band 0} answers {u16 playback_ch, u16 capture_ch}, unique per
 * model (bar the Red twins).
 *
 * Everything downstream is sized from c->model, so a wrong model would stream
 * the wrong width into wrongly strided rings. An unknown geometry therefore
 * refuses to register, and the probe error prints the pair to add to the
 * table.
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
 * copied out under mbox_lock, so a command from the meter worker cannot
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

/*
 * Ask the device who it is (STREAM_INFO{band 0}). Returns NULL if the query
 * fails or the pair matches no known model; either is fatal to probe, and
 * *collapsed says which: true = the read failed or was refused (usually "not
 * ready yet, retry"), false = a valid reply with an unknown geometry (new
 * hardware, which needs a clarett_model entry).
 */
const struct clarett_model *
clarett_detect_model(struct clarett *c, bool *collapsed, bool quiet)
{
	static const struct clarett_model *const models[] = {
		&clarett_2pre, &clarett_4pre, &clarett_8pre, &clarett_8prex,
		&red_8line, &red_16line, &red_4pre, &red_8pre,
	};
	u16 pb, cap;
	int i;

	/* when not quiet, every failure logs the raw response */
	*collapsed = clarett_stream_info(c, 0, &pb, &cap, quiet) != 0;
	if (*collapsed)
		return NULL;

	/*
	 * The pair identifies the model, except for the two Red pairs that
	 * share one (4Pre/8Line, 8Pre/16Line). Those carry tb_gen, and the
	 * unit's own Thunderbolt controller decides; if it cannot be told, the
	 * first model with the pair is taken (clarett_log_twin() says so).
	 */
	{
		const struct clarett_model *first = NULL;
		int gen = clarett_unit_tb_gen(c->pci);

		for (i = 0; i < ARRAY_SIZE(models); i++) {
			if (models[i]->playback_channels != pb ||
			    models[i]->capture_channels != cap)
				continue;
			if (!first)
				first = models[i];
			if (gen && models[i]->tb_gen == gen)
				return models[i];
		}
		if (first)
			return first;
	}

	if (!quiet)
		dev_warn(&c->pci->dev,
			 "model auto-detect: unrecognized stream geometry (playback=%u capture=%u) — unknown model, needs a clarett_model entry\n",
			 pb, cap);
	return NULL;
}

/*
 * For a model with a geometry twin, say how it was told apart: one line, so a
 * first report from a unit names the controller and the FPGA word (register
 * 0x000) that confirm or correct the rule.
 */
void clarett_log_twin(struct clarett *c)
{
	const struct clarett_model *m = c->model;
	struct pci_dev *br = pci_upstream_bridge(c->pci);
	int gen = clarett_unit_tb_gen(c->pci);

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
 * Read how many leading capture channels the device writes at each speed band
 * into c->rx_live[]: STREAM_INFO's capture word at bands 1 and 2 (band 0 is the
 * full width by definition). The device's answer is used. The model table
 * (rx_live_mid/high) is the fallback for a band the device does not answer, or
 * answers with a width outside the frame, and a cross-check: a disagreement is
 * logged, so a table value never read from a unit is checked the first time one
 * attaches. Run after the model is final, since the Red twins share band 0 but
 * not necessarily the table.
 */
void clarett_read_rx_live(struct clarett *c)
{
	const struct clarett_model *m = c->model;
	u8 width = m->capture_channels;
	unsigned int band;

	c->rx_live[0] = width;
	for (band = 1; band < CLARETT_SPEED_BANDS; band++) {
		u8 table = band == 1 ? m->rx_live_mid : m->rx_live_high;
		u16 pb, cap;

		if (!table || table > width)
			table = width;
		c->rx_live[band] = table;
		if (clarett_stream_info(c, band, &pb, &cap, false)) {
			dev_warn(&c->pci->dev,
				 "STREAM_INFO{%u} unanswered: using the %s table's %u live capture channels\n",
				 band, m->name, table);
			continue;
		}
		if (!cap || cap > width) {
			dev_warn(&c->pci->dev,
				 "STREAM_INFO{%u}: capture width %u outside 1..%u; using the %s table's %u\n",
				 band, cap, width, m->name, table);
			continue;
		}
		if (cap != table)
			dev_warn(&c->pci->dev,
				 "STREAM_INFO{%u}: device writes %u capture channels, the %s table says %u; using the device's\n",
				 band, cap, m->name, table);
		c->rx_live[band] = cap;
	}
	dev_dbg(&c->pci->dev,
		"capture channels live per speed band: %u/%u/%u\n",
		c->rx_live[0], c->rx_live[1], c->rx_live[2]);
}
