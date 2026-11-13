// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — PCI bring-up, probe and
 * removal.
 *
 * Supports the Clarett Thunderbolt line (2Pre / 4Pre / 8Pre / 8PreX) and the
 * Red range (8Line), detected at probe. Provides the mixer control plane
 * (through the FCP hwdep + fcp-server), PCM capture and playback, and DIN MIDI.
 *
 * Clean-room: built from black-box observation of the device's host interface
 * and Focusrite's device XML descriptors; no vendor driver code was used.
 */
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/kthread.h>
/* sched_set_fifo_low() — real-time priority for the stream servicer */
#include <linux/sched.h>
#include <linux/bitmap.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/jiffies.h>
#include <sound/core.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/initval.h>
#include "clarett.h"

#define PCI_VENDOR_FOCUSRITE   0x1cb5
#define PCI_DEVICE_CLARETT     0x0002

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;
module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for the Clarett/Red sound card.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for the Clarett/Red sound card.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable the Clarett/Red sound card.");

/*
 * Which index[]/id[]/enable[] entry a card uses: the lowest one no live card
 * holds. The usual count of probes would run out, since a Thunderbolt unit can
 * attach and detach many times per boot. An entry is released only when its
 * card is freed (clarett_card_free()), so a unit re-attached while its old card
 * is still open takes the next entry. A unit refused by enable[] does not keep
 * its entry.
 */
static DEFINE_MUTEX(clarett_slot_lock);
static DECLARE_BITMAP(clarett_slots, SNDRV_CARDS);

static int clarett_get_slot(void)
{
	int slot;

	mutex_lock(&clarett_slot_lock);
	slot = find_first_zero_bit(clarett_slots, SNDRV_CARDS);
	if (slot < SNDRV_CARDS)
		__set_bit(slot, clarett_slots);
	mutex_unlock(&clarett_slot_lock);
	return slot < SNDRV_CARDS ? slot : -ENODEV;
}

static void clarett_put_slot(int slot)
{
	mutex_lock(&clarett_slot_lock);
	__clear_bit(slot, clarett_slots);
	mutex_unlock(&clarett_slot_lock);
}

/*
 * THE DRIVER NEVER ARMS. A device that has been set up once restores its
 * session from flash at power-up, and a host-side bring-up would reset the
 * user's routing and mixer to the factory default. Probe waits for that session
 * to answer and detects the model from it. If it never answers within
 * CLARETT_READY_TIMEOUT_MS, probe fails rather than register a placeholder;
 * power-cycle the unit to retry.
 */

/*
 * Probe readiness. The device acknowledges the response-buffer address (the
 * 0x414 write in clarett_hw_init) by raising 0x400 bit0, and a command sent
 * before that is never answered. The time varies by model, from ~0.15 ms (Red
 * 8Line) to ~13 ms (8Pre, 8PreX) cold, so clarett_hw_init waits for the
 * acknowledgement (up to CLARETT_ADDR_ACK_MS). No register shows readiness.
 *
 * Fallback, for a device that does not acknowledge in time: a command sent too
 * early is never answered, and the mailbox then starts over at seq 0, which the
 * device does answer. So probe re-asks at once until it answers or
 * CLARETT_READY_TIMEOUT_MS runs out, each attempt waiting
 * CLARETT_READY_RESP_MS (a response that lands at all lands within a
 * millisecond).
 */

/* response-address acknowledgement wait */
#define CLARETT_ADDR_ACK_MS	500
/* each fallback attempt's response deadline */
#define CLARETT_READY_RESP_MS	20
/* give up on a device that has not answered by then */
#define CLARETT_READY_TIMEOUT_MS	10000

/*
 * After clarett_hw_teardown() the BAR mapping may be gone: writes are dropped
 * and reads return all ones, as a removed device's do.
 */
void clarett_wl(struct clarett *c, u32 off, u32 val)
{
	if (READ_ONCE(c->hw_gone))
		return;
	writel(val, c->bar0 + off);
}

u32 clarett_rl(struct clarett *c, u32 off)
{
	if (READ_ONCE(c->hw_gone))
		return ~0u;
	return readl(c->bar0 + off);
}

void clarett_hw_init(struct clarett *c)
{
	ktime_t t0;

	/*
	 * Pre-mailbox init: the two 0x510/0x500 writes every attach makes
	 * (device open; not decoded beyond these values), the interrupt-cause
	 * enable, then the response-buffer address, waiting for the device to
	 * acknowledge it. The enable goes first because the acknowledgement
	 * arrives as an interrupt. Sleeps.
	 */
	clarett_wl(c, 0x510, 0x8);
	clarett_wl(c, REG_MIDI_STATUS, 0x8);
	clarett_wl(c, REG_IRQ0_ENABLE, 0xf000003f);
	clarett_wl(c, REG_DMA_ADDR_LO, lower_32_bits(c->resp_dma));
	reinit_completion(&c->addr_acked);
	WRITE_ONCE(c->addr_ack_wait, true);
	t0 = ktime_get();
	clarett_wl(c, REG_DMA_ADDR_HI, upper_32_bits(c->resp_dma));
	if (wait_for_completion_timeout(&c->addr_acked,
					msecs_to_jiffies(CLARETT_ADDR_ACK_MS)))
		dev_dbg(&c->pci->dev,
			"readiness: response address acknowledged after %lld us\n",
			ktime_us_delta(ktime_get(), t0));
	else
		dev_warn(&c->pci->dev,
			 "response address not acknowledged within %d ms; asking anyway\n",
			 CLARETT_ADDR_ACK_MS);
	WRITE_ONCE(c->addr_ack_wait, false);

	memset(c->shadow, 0, sizeof(c->shadow));
}

/*
 * Read the unit's serial from flash: the App_Env segment (found by name) begins
 * with one ASCII line, "SERIAL_NO = <digits>", optionally newline-terminated,
 * and is otherwise erased. Read-only (FLASH_INFO, SEGMENT_INFO, FLASH_READ),
 * and only on the first read of the "serial" sysfs attribute, never at probe,
 * so a slow flash cannot stall an attach. Leaves c->serial empty on failure.
 */
static void clarett_read_serial(struct clarett *c)
{
	u8 req[12], info[24], line[64];
	u32 count, i;
	const char *p, *end;
	size_t n = 0;

	c->serial[0] = '\0';
	if (clarett_fcp_cmd(c, FCP_FLASH_INFO, NULL, 0, info, 12))
		return;
	/* {u32 total size, u32 segment count, u8[4]} */
	count = clarett_get_le32(info + 4);
	/* implausible: not a segment table */
	if (!count || count > 32)
		return;
	for (i = 0; i < count; i++) {
		clarett_put_le32(req, i);
		if (clarett_fcp_cmd(c, FCP_FLASH_SEGMENT_INFO, req, 4, info,
				    sizeof(info)))
			return;
		if (!strncmp((const char *)info + 8, "App_Env", 16))
			break;
	}
	if (i == count)
		return;
	clarett_put_le32(req, i);
	clarett_put_le32(req + 4, 0);
	clarett_put_le32(req + 8, sizeof(line));
	if (clarett_fcp_cmd(c, FCP_FLASH_READ, req, sizeof(req), line,
			    sizeof(line)))
		return;

	end = (const char *)line + sizeof(line);
	p = strnstr((const char *)line, "SERIAL_NO", sizeof(line));
	if (!p)
		return;
	for (p += strlen("SERIAL_NO"); p < end && (*p == ' ' || *p == '='); p++)
		;
	/*
	 * ASCII only: the kernel's isalnum() follows Latin-1, where the erased
	 * flash after an unterminated line (0xff) is a letter.
	 */
	while (p + n < end && n < CLARETT_SERIAL_LEN - 1 &&
	       isascii(p[n]) && isalnum(p[n]))
		n++;
	memcpy(c->serial, p, n);
	c->serial[n] = '\0';
}

/*
 * Seed the whole config shadow from the device, so the command-3 enable bytes
 * (72/73) have their real values for a safe read-modify-write of their packed
 * bits. The first GET after programming the DMA address can come back empty,
 * so retry briefly; a refusing device returns size=0 and no payload, so check
 * the size too.
 */
int clarett_seed_shadow(struct clarett *c)
{
	u8 req[8], buf[CLARETT_CONFIG_SIZE];
	u16 size;
	int err, attempt;

	clarett_put_le32(req, 0);
	clarett_put_le32(req + 4, CLARETT_CONFIG_SIZE);
	for (attempt = 0; attempt < 3; attempt++) {
		/* copied out under mbox_lock: other commands may be running */
		err = clarett_fcp_read(c, FCP_GET_DATA, req, sizeof(req), buf,
				       sizeof(buf), &size, NULL);
		if (err)
			return err;

		if (size >= CLARETT_CONFIG_SIZE) {
			memcpy(c->shadow, buf, CLARETT_CONFIG_SIZE);
			/*
			 * Only 24/28/112 are known to read back live; mark just
			 * those known. The rest of the seed — preamp Mode/Air
			 * in particular — is not device-reported, so it stays
			 * unknown and its first write goes through.
			 */
			set_bit(24,  c->shadow_known);
			set_bit(28,  c->shadow_known);
			set_bit(112, c->shadow_known);
			return 0;
		}
	}
	return -EIO;
}

/*
 * Make the two monitor outputs follow the monitor section's Mute/Dim. Without
 * these command-3 enable bits, writing the global Mute (24) / Dim (28) flips
 * the master flag but no output obeys it. RMW from the seeded shadow so the
 * other outputs' enable bits are kept; idempotent — clarett_write_bits()
 * no-ops if the bits are already set.
 */
int clarett_enable_monitor_hw_controls(struct clarett *c)
{
	int err;

	err = clarett_write_bits(c, HWEN_MUTE_OFFSET, HWEN_MONITOR_MUTE_MASK,
				 HWEN_MONITOR_MUTE_MASK, HWEN_ACTIVATE);
	if (err)
		return err;
	return clarett_write_bits(c, HWEN_DIM_OFFSET, HWEN_MONITOR_DIM_MASK,
				  HWEN_MONITOR_DIM_MASK, HWEN_ACTIVATE);
}

/* vec0: the 0x400 command phase and device events. */
static void clarett_irq_event(struct clarett *c)
{
	bool inflight = atomic_read(&c->cmd_inflight);
	u32 cause, ev;

	/*
	 * This is the only reader of 0x400 (read-to-clear), once per vec0 MSI,
	 * in or out of a command: a second reader would take bits this one
	 * needs. The phase bits are recorded for the command in flight; the
	 * event bits go to the relay either way.
	 */
	/*
	 * 0x100 FIRST when a command is in flight: its DONE bit (bit29)
	 * reflects "0x400 has bits pending", so reading 0x400 first
	 * clears what DONE reports and the command times out. The read
	 * clears DONE; the command itself completes on the phase bits
	 * below.
	 */
	if (inflight)
		readl(c->bar0 + REG_IRQ0_CAUSE); /* read-to-clear */
	cause = readl(c->bar0 + REG_NOTIFY_CAUSE);
	ev = cause & NOTIFY_EVENT_MASK;

	if (inflight) {
		atomic_or(cause & NOTIFY_PHASE_MASK, &c->mbox_phase);
		if (cause & NOTIFY_REQ_ACCEPTED)
			complete(&c->mbox_accepted);
		if (cause & NOTIFY_RESP_LANDED)
			complete(&c->mbox_landed);
	}

	/*
	 * Init is waiting for the response-address acknowledgement
	 * (CLARETT_ADDR_ACK_MS).
	 */
	if (READ_ONCE(c->addr_ack_wait) &&
	    (cause & NOTIFY_REQ_ACCEPTED)) {
		WRITE_ONCE(c->addr_ack_wait, false);
		complete(&c->addr_acked);
	}

	/*
	 * Only the event bits are a notification. ctl_ready gates the relay:
	 * the handlers are hooked before the controls exist, and an event
	 * then must not notify. An audio period raises no event bit, so a
	 * stream adds no relays.
	 */
	if (ev && READ_ONCE(c->ctl_ready)) {
		atomic_or(ev, &c->notify_bits);
		schedule_work(&c->notify_work);
	}
}

/*
 * MSI handler, one Linux IRQ per vector (dev_id). vec0 carries the mailbox's
 * command phase and the device events (clarett_irq_event()). vec1/vec2 are the
 * stream period vectors; the servicer polls the period blocks, so here they are
 * only read. MSI is edge-triggered, so an uncleared cause cannot storm. Every
 * vector returns IRQ_HANDLED so the core does not disable it as spurious.
 */
static irqreturn_t clarett_irq(int irq, void *dev_id)
{
	struct clarett_irqctx *ic = dev_id;
	struct clarett *c = ic->c;

	/*
	 * Which vector signals MIDI RX is not known, so drain on every one;
	 * that also keeps an undrained FIFO from livelocking a vector.
	 */
	if (READ_ONCE(c->rmidi))
		clarett_midi_irq(c);

	if (ic->idx == CLARETT_VEC_EVENT) {
		clarett_irq_event(c);
	/* data-plane period IRQs */
	} else if (ic->idx == 1 || ic->idx == 2) {
		/* read-to-clear */
		readl(c->bar0 + (ic->idx == 1 ? STREAM_BLK0 : STREAM_BLK1));
	}
	return IRQ_HANDLED;
}

/*
 * Allocate MSI vectors. Done early, so MSI is enabled in config space BEFORE
 * the first BAR access, as the device sees on a normal attach. The handlers
 * hook later (clarett_setup_irq); an unhandled MSI in between is harmless.
 */
static int clarett_enable_msi(struct clarett *c)
{
	struct pci_dev *pci = c->pci;
	int nvec;

	/*
	 * Accept as few as 1 vector: the device then funnels every cause to
	 * the lower vectors, and clarett_irq reads the cause registers anyway.
	 */
	nvec = pci_alloc_irq_vectors(pci, 1, CLARETT_NUM_VECTORS, PCI_IRQ_MSI);
	if (nvec < 0) {
		dev_err(&pci->dev, "MSI allocation failed (%d)\n", nvec);
		return nvec;
	}
	c->n_vec = nvec;
	/* a short count is worth a warn; the expected 4/4 is debug */
	if (nvec < CLARETT_NUM_VECTORS)
		dev_warn(&pci->dev,
			 "MSI: got %d/%d vectors (causes funnel to the allocated ones)\n",
			 nvec, CLARETT_NUM_VECTORS);
	else
		dev_dbg(&pci->dev, "MSI: got %d/%d vectors\n", nvec,
			CLARETT_NUM_VECTORS);
	return 0;
}

/*
 * Hook the handlers onto the allocated vectors. Required: vec0 completes every
 * mailbox command. On failure the vectors stay allocated, and the probe error
 * path frees them (clarett_teardown_irq).
 */
static int clarett_setup_irq(struct clarett *c)
{
	struct pci_dev *pci = c->pci;
	int i, err;

	for (i = 0; i < c->n_vec; i++) {
		c->irq_ctx[i].c = c;
		c->irq_ctx[i].idx = i;
		err = request_irq(pci_irq_vector(pci, i), clarett_irq, 0,
				  KBUILD_MODNAME, &c->irq_ctx[i]);
		if (err) {
			dev_err(&pci->dev, "request_irq vec %d failed (%d)\n",
				i, err);
			while (--i >= 0)
				free_irq(pci_irq_vector(pci, i),
					 &c->irq_ctx[i]);
			return err;
		}
	}
	c->irq_ready = true;
	return 0;
}

static void clarett_teardown_irq(struct clarett *c)
{
	int i;

	if (c->irq_ready)
		for (i = 0; i < c->n_vec; i++)
			free_irq(pci_irq_vector(c->pci, i), &c->irq_ctx[i]);
	if (c->n_vec)
		pci_free_irq_vectors(c->pci);
	c->irq_ready = false;
	c->n_vec = 0;
}

/*
 * Everything that touches the device or its devres-managed resources (BAR
 * mapping, response buffer, stream rings, MSI vectors), done while they still
 * exist: from remove(), before it returns and devres frees them, or from
 * clarett_card_free() on a probe error. Runs once.
 *
 * remove() cannot leave this to the card's last close: a unit removed during
 * system resume (a Thunderbolt chain lost across the sleep) is removed while
 * userspace is still frozen, so waiting there for fcp-server and alsactl to
 * close their handles would deadlock the resume.
 *
 * Mailbox commands still run the normal MSI-paced cycle until the IRQs are
 * freed, so a pending settings save is flushed first; on a device that is
 * already gone it fails at once.
 */
static void clarett_hw_teardown(struct clarett *c)
{
	if (c->hw_gone)
		return;
	/* stop the ISR queueing new notify work */
	WRITE_ONCE(c->ctl_ready, false);
	cancel_delayed_work_sync(&c->resume_work);
	cancel_delayed_work_sync(&c->follow_work);
	if (cancel_delayed_work_sync(&c->save_work))
		clarett_data_cmd(c, FCP_ACTIVATE_PERSIST);
	cancel_work_sync(&c->notify_work);
	/* cancel the MIDI TX drain */
	clarett_midi_stop(c);
	/* halt streaming DMA before devres frees the ring */
	clarett_engine_stop(c);
	if (c->bar0)
		/* mask causes before freeing handlers */
		writel(0, c->bar0 + REG_IRQ0_ENABLE);
	/* syncs any in-flight ISR */
	clarett_teardown_irq(c);
	/*
	 * An ISR already past its ctl_ready check when the flag flipped may
	 * have queued one more notify after the cancel above; re-cancel now
	 * that free_irq has synced them all (a straggler that already started
	 * finds the mailbox refusing, as irq_ready is now clear — harmless).
	 */
	cancel_work_sync(&c->notify_work);
	/* notify_work may have queued one */
	cancel_delayed_work_sync(&c->follow_work);

	/*
	 * Taking each lock waits out whoever holds it: an in-flight TX-ring
	 * write, mailbox command, meter read or meter-map ioctl finishes, and
	 * every later one sees hw_gone.
	 */
	mutex_lock(&c->pcm_lock);
	WRITE_ONCE(c->hw_gone, true);
	mutex_unlock(&c->pcm_lock);
	mutex_lock(&c->mbox_lock);
	mutex_unlock(&c->mbox_lock);
	mutex_lock(&c->hwdep_lock);
	mutex_unlock(&c->hwdep_lock);

	clarett_quiesce_dma(c->pci, c->bar0);
}

/*
 * card->private_free: runs inside snd_card_free AFTER userspace is disconnected
 * and the last file handle has closed, but before c (card private data) is
 * freed. After a remove() that is any time later, and only software state is
 * left: clarett_hw_teardown() has run. On a probe error it runs the teardown
 * itself. Must tolerate a partially initialized card (probe error paths reach
 * here through the same snd_card_free).
 */
static void clarett_card_free(struct snd_card *card)
{
	struct clarett *c = card->private_data;

	clarett_hw_teardown(c);
	/*
	 * A hwdep ioctl that was in flight at remove may have queued work
	 * since; the mailbox now refuses it, but the items must not outlive c.
	 */
	WRITE_ONCE(c->ctl_ready, false);
	cancel_delayed_work_sync(&c->resume_work);
	/* nothing to flush to: the device is gone */
	cancel_delayed_work_sync(&c->save_work);
	cancel_work_sync(&c->notify_work);
	cancel_delayed_work_sync(&c->follow_work);
	clarett_midi_stop(c);
	/*
	 * Last, because notify_work is what re-arms it: the debounced hwdep
	 * relay wake, whose timer must not outlive c.
	 */
	clarett_hwdep_free(c);
	clarett_hwdep_free_meters(c);
	clarett_put_slot(c->card_slot);
}

/*
 * /proc/asound/cardN/clarett: the model's name and the current sample rate,
 * which fcp-server reads to pick the meter layout for the speed. The
 * machine-readable model key is the card's "Clarett:<slug>" component.
 */
static void clarett_proc_read(struct snd_info_entry *entry,
			      struct snd_info_buffer *buf)
{
	struct clarett *c = entry->private_data;

	snd_iprintf(buf, "model: %s\n", c->model->name);
	snd_iprintf(buf, "rate: %u\n", READ_ONCE(c->cur_rate));
}

/* The software state probe needs before it touches the device. */
static void clarett_init_state(struct clarett *c, struct snd_card *card,
			       struct pci_dev *pci,
			       const struct pci_device_id *ent)
{
	c->card = card;
	c->pci = pci;
	c->clock_src = CLARETT_CLOCK_INTERNAL;
	/*
	 * The whole line shares the PCI id, so the id_table's model is a
	 * placeholder until clarett_identify(). Size nothing from it before.
	 */
	c->model = (const struct clarett_model *)ent->driver_data;
	mutex_init(&c->mbox_lock);
	mutex_init(&c->hwdep_lock);
	mutex_init(&c->pcm_lock);
	mutex_init(&c->serial_lock);
	init_waitqueue_head(&c->hwdep_notify_wait);
	init_completion(&c->mbox_accepted);
	init_completion(&c->mbox_landed);
	init_completion(&c->addr_acked);
	INIT_WORK(&c->notify_work, clarett_notify_work);
	INIT_DELAYED_WORK(&c->save_work, clarett_save_work);
	INIT_DELAYED_WORK(&c->follow_work, clarett_follow_work);
	INIT_DELAYED_WORK(&c->resume_work, clarett_resume_work);
	atomic_set(&c->notify_bits, 0);
	atomic_set(&c->cmd_inflight, 0);
	atomic_set(&c->mbox_phase, 0);
	/*
	 * Teardown lives in private_free so snd_card_free sequences it after
	 * the last userspace handle closes but before c is freed (works are all
	 * INIT'd above).
	 */
	card->private_free = clarett_card_free;
}

/* Enable the PCI device, map BAR0 and allocate the response buffer. */
static int clarett_map_device(struct clarett *c)
{
	struct pci_dev *pci = c->pci;
	int err;

	err = pcim_enable_device(pci);
	if (err)
		return err;

	err = pcim_iomap_regions(pci, BIT(CLARETT_BAR), KBUILD_MODNAME);
	if (err)
		return err;
	c->bar0 = pcim_iomap_table(pci)[CLARETT_BAR];

	pci_set_master(pci);

	err = dma_set_mask_and_coherent(&pci->dev, DMA_BIT_MASK(32));
	if (err)
		return err;

	c->resp_size = PAGE_SIZE;
	c->resp_buf = dmam_alloc_coherent(&pci->dev, c->resp_size,
					  &c->resp_dma, GFP_KERNEL);
	if (!c->resp_buf)
		return -ENOMEM;
	/* the response buffer's bus address (0x414 carries the high word) */
	dev_dbg(&pci->dev, "resp buffer dma addr %pad (0x414 high word 0x%x)\n",
		&c->resp_dma, upper_32_bits(c->resp_dma));
	return 0;
}

/*
 * Wait for the flash-restored session to answer and identify the model (see
 * the note on arming above).
 */
static int clarett_identify(struct clarett *c)
{
	struct pci_dev *pci = c->pci;
	bool collapsed = false;
	const struct clarett_model *det;
	unsigned long deadline =
		jiffies + msecs_to_jiffies(CLARETT_READY_TIMEOUT_MS);
	int tries = 0;

	/*
	 * Ask until the device answers (see the probe readiness note above),
	 * quietly; the last attempt is re-run with logging below. The budget is
	 * short because a removal of the device waits for probe.
	 */
	c->resp_deadline_ms = CLARETT_READY_RESP_MS;
	for (;;) {
		tries++;
		det = clarett_detect_model(c, &collapsed, true);
		if (det || !collapsed || time_after(jiffies, deadline))
			break;
		clarett_mbox_clear_causes(c);
	}
	c->resp_deadline_ms = 0;
	if (tries > 1)
		dev_dbg(&pci->dev, "readiness: answered on attempt %d\n",
			tries);
	/*
	 * One logged pass on any failure: a refusal's status/size, or an
	 * unmatched geometry.
	 */
	if (!det)
		det = clarett_detect_model(c, &collapsed, false);

	if (collapsed) {
		/* never became ready: fail, don't register a placeholder */
		dev_err(&pci->dev,
			"device did not become ready within %d ms over %d attempts (mailbox %s) — refusing to register; power-cycle the unit to retry.\n",
			CLARETT_READY_TIMEOUT_MS, tries,
			c->mbox_wedged ?
			"wedged: no response, or one echoing another command's seq" :
			"answering, but refusing the request");
		return -ENODEV;
	}

	if (!det) {
		/*
		 * A clean answer with a geometry no clarett_model claims:
		 * refuse rather than size the rings for different hardware. The
		 * warn above carries the pair to add to the table.
		 */
		dev_err(&pci->dev,
			"unknown Clarett model (stream geometry matches none in the table) — refusing to register. Add a clarett_model entry for the playback/capture pair logged above.\n");
		return -ENODEV;
	}
	c->model = det;
	clarett_log_twin(c);
	clarett_read_rx_live(c);
	/* Nothing arms the device, so its flash-persisted routing stands. */
	return 0;
}

/*
 * Model-dependent setup once the model is known. Returns the shadow seed's
 * result: 0 if the config shadow was seeded from the device.
 */
static int clarett_setup_model(struct clarett *c)
{
	struct pci_dev *pci = c->pci;
	int err, seeded;
	u8 r[4];

	/*
	 * Fragment slot strides. A fragment that straddles a 4 KB page is
	 * mis-framed by the device's DMA: packed contiguously, capture drifts
	 * its channel alignment by 8 bytes per page, and playback on a model
	 * whose fragment is not a power of two is garbled. So each fragment
	 * gets its own slot of the next power of two, which divides the page,
	 * over a page-aligned area.
	 */
	c->rx_slot = clarett_frag_bytes(c->model->capture_channels);
	c->rx_slot = roundup_pow_of_two(c->rx_slot);
	c->tx_slot = clarett_tx_frag_bytes(c->model->playback_channels);
	c->tx_slot = roundup_pow_of_two(c->tx_slot);

	/*
	 * Seed the shadow from the device so the enable-byte RMW below is safe.
	 * Best-effort: if it fails we skip the enable writes.
	 */
	seeded = clarett_seed_shadow(c);
	if (seeded)
		dev_warn(&pci->dev,
			 "config shadow seed failed (%d); leaving hardware mute/dim enables untouched\n",
			 seeded);

	/*
	 * Make Mute/Dim affect Monitor Out 1-2 (the per-output enable bits);
	 * needed whoever drives the Mute/Dim bytes. Only with a seeded shadow,
	 * for a correct read-modify-write.
	 */
	if (!seeded) {
		err = clarett_enable_monitor_hw_controls(c);
		if (err)
			dev_warn(&pci->dev,
				 "could not enable monitor hardware mute/dim (%d)\n",
				 err);
	}

	/*
	 * Seed the published sample rate from the device, which keeps its rate
	 * across driver reloads, so /proc/asound/cardN/clarett is right before
	 * anything streams. The CONFIGURED rate, not the measured one: on an
	 * external source with nothing locked, FCP_SYNC_RATE reports 192000
	 * while the routing and meter tables follow the configured rate.
	 */
	if (!clarett_fcp_cmd(c, FCP_SYNC_RATE_CFG, NULL, 0, r, sizeof(r)))
		WRITE_ONCE(c->cur_rate, clarett_get_le32(r));
	if (!READ_ONCE(c->cur_rate))
		WRITE_ONCE(c->cur_rate, CLARETT_DEFAULT_RATE);
	return seeded;
}

/*
 * The card's userspace-facing interfaces. Each is optional: a failure is
 * logged and the card comes up without it. Returns whether the PCM fully
 * registered.
 */
static bool clarett_create_interfaces(struct clarett *c)
{
	struct pci_dev *pci = c->pci;
	bool pcm_ok = false;
	int err;

	/* Controls live in userspace: expose the FCP hwdep for fcp-server. */
	err = clarett_hwdep_init(c);
	if (err)
		dev_warn(&pci->dev, "FCP hwdep create failed (%d)\n", err);

	/*
	 * The one exception to "controls live in userspace": the clock source
	 * is not a config byte, so fcp-server cannot map it, and SET_CLOCK
	 * needs the sample rate that only this driver knows.
	 */
	err = clarett_add_clock_control(c);
	if (err)
		dev_warn(&pci->dev, "Clock Source control create failed (%d)\n",
			 err);

	/* controls exist; the ISR notify path may fire */
	WRITE_ONCE(c->ctl_ready, true);
	/* sync the stored SW gains of HW outputs to the knob */
	clarett_schedule_follow(c);

	/* PCM */
	err = clarett_create_pcm(c);
	if (err)
		dev_warn(&pci->dev,
			 "PCM create failed (%d); continuing without PCM\n",
			 err);
	else
		/* c->pcm is set before an alloc that can still fail */
		pcm_ok = true;

	/* DIN MIDI, on models with MIDI jacks (a no-op otherwise) */
	err = clarett_create_midi(c);
	if (err)
		dev_warn(&pci->dev,
			 "MIDI create failed (%d); continuing without MIDI\n",
			 err);
	return pcm_ok;
}

/* The card's driver, names and components string. */
static int clarett_name_card(struct clarett *c)
{
	struct snd_card *card = c->card;
	char comp[48];

	strscpy(card->driver, "Clarett", sizeof(card->driver));
	/*
	 * snd-usb-audio's naming: the brand-free product name as the shortname
	 * (api.alsa.card.name), the manufacturer only in the longname.
	 */
	snprintf(card->shortname, sizeof(card->shortname), "%s",
		 c->model->name);
	snprintf(card->longname, sizeof(card->longname), "Focusrite %s at %s",
		 c->model->name, pci_name(c->pci));

	/*
	 * The model slug, in the card's components string
	 * ("Clarett:clarett-8prex"), where snd-usb-audio puts a USB card's
	 * "USB<vid>:<pid>": readable through the control API
	 * (snd_ctl_card_info_get_components(), `alsactl info`) and by UCM as
	 * ${CardComponents}. fcp-server keys its per-model maps on it.
	 */
	snprintf(comp, sizeof(comp), "Clarett:%s", c->model->slug);
	return snd_component_add(card, comp);
}

/*
 * The probe summary: on a healthy load, the ONLY thing this driver prints. The
 * detail is at dev_dbg; bring it back at runtime with dynamic debug, e.g.
 *   echo 'module snd_clarett +p' >/sys/kernel/debug/dynamic_debug/control
 * or match one statement by its format string.
 */
static void clarett_log_summary(struct clarett *c, int seeded, bool pcm_ok)
{
	struct pci_dev *pci = c->pci;
	char feat[64];
	int n = 0;

	if (c->hwdep_ready)
		n += scnprintf(feat + n, sizeof(feat) - n, "%sFCP hwdep",
			       n ? ", " : "");
	if (pcm_ok)
		n += scnprintf(feat + n, sizeof(feat) - n, "%sPCM %u/%uch",
			       n ? ", " : "", c->model->playback_channels,
			       c->model->capture_channels);
	if (READ_ONCE(c->rmidi))
		n += scnprintf(feat + n, sizeof(feat) - n, "%sMIDI",
			       n ? ", " : "");
	if (!n)
		scnprintf(feat, sizeof(feat), "no interfaces registered");

	/*
	 * The firmware versions come from the seeded shadow, so they are left
	 * out if the seed failed. The serial is never printed (kernel logs get
	 * pasted into public bug reports); it is in sysfs.
	 */
	if (!seeded) {
		const u8 *s = c->shadow;
		char fw[48];

		scnprintf(fw, sizeof(fw), "App %u, FPGA %u",
			  clarett_get_le32(s + APPSPACE_FW_APP),
			  clarett_get_le32(s + APPSPACE_FW_FPGA) & 0xffff);
		if (c->model->fw_front) {
			size_t len = strlen(fw);

			scnprintf(fw + len, sizeof(fw) - len, ", Front %u",
				  clarett_get_le32(s + APPSPACE_FW_FRONT));
		}
		dev_info(&pci->dev, "%s: %s; %s\n", c->model->name, fw, feat);
	} else {
		dev_info(&pci->dev, "%s: %s\n", c->model->name, feat);
	}
}

static int clarett_probe(struct pci_dev *pci, const struct pci_device_id *ent)
{
	struct snd_card *card;
	struct clarett *c;
	bool pcm_ok;
	void __iomem *bar0;
	int err, seeded, slot;

	slot = clarett_get_slot();
	if (slot < 0)
		return slot;
	if (!enable[slot]) {
		clarett_put_slot(slot);
		return -ENOENT;
	}
	err = snd_card_new(&pci->dev, index[slot], id[slot], THIS_MODULE,
			   sizeof(*c), &card);
	if (err < 0) {
		clarett_put_slot(slot);
		return err;
	}

	c = card->private_data;
	/* released by clarett_card_free(), from here on the only free path */
	c->card_slot = slot;
	clarett_init_state(c, card, pci, ent);

	err = clarett_map_device(c);
	if (err)
		goto err_free;

	/*
	 * MSI is enabled in config space before the first BAR access, so the
	 * device never sees a session start from a host without an interrupt
	 * path. Handlers hook immediately too: mailbox completion is MSI-paced
	 * from the first command (the notify path stays gated on ctl_ready).
	 */
	err = clarett_enable_msi(c);
	if (err)
		goto err_free;
	err = clarett_setup_irq(c);
	if (err)
		goto err_free;

	clarett_hw_init(c);

	err = clarett_identify(c);
	if (err)
		goto err_free;

	seeded = clarett_setup_model(c);
	pcm_ok = clarett_create_interfaces(c);

	err = clarett_name_card(c);
	if (err < 0)
		goto err_free;

	/*
	 * /proc/asound/cardN/clarett (clarett_proc_read). Not fatal: a failure
	 * only costs fcp-server its per-rate meter layout.
	 */
	if (snd_card_ro_proc_new(card, "clarett", c, clarett_proc_read))
		dev_warn(&pci->dev,
			 "could not create /proc/asound/.../clarett model entry\n");

	err = snd_card_register(card);
	if (err)
		goto err_free;

	pci_set_drvdata(pci, card);
	clarett_log_summary(c, seeded, pcm_ok);
	return 0;

err_free:
	/* may still be NULL; c is freed by snd_card_free */
	bar0 = c->bar0;
	/* clarett_card_free() runs the full teardown */
	snd_card_free(card);
	clarett_quiesce_dma(pci, bar0);
	return err;
}

static void clarett_remove(struct pci_dev *pci)
{
	struct snd_card *card = pci_get_drvdata(pci);
	struct clarett *c = card->private_data;

	/*
	 * Stop the stream servicer FIRST. Freeing the card frees the PCM
	 * devices, and each substream's buffer, before card->private_free
	 * runs, and a servicer still ticking would write into a freed buffer:
	 * a unit powered off mid-stream leaves it running. pcm_lock does not
	 * help, as it serialises the tick only against the driver's own
	 * hw_free.
	 */
	WRITE_ONCE(c->ctl_ready, false); /* no new notify work from the ISR */
	cancel_delayed_work_sync(&c->resume_work);
	cancel_delayed_work_sync(&c->follow_work);
	clarett_engine_stop(c);

	/*
	 * Disconnect, then wake fcp-server's read()/poll() on
	 * hwdep_notify_wait, whose wait re-checks card->shutdown. New file
	 * operations now fail.
	 */
	snd_card_disconnect(card);
	wake_up_interruptible(&c->hwdep_notify_wait);
	/*
	 * Tear the hardware side down while the BAR mapping, the buffers and
	 * the vectors still exist: devres frees them as soon as this returns.
	 */
	clarett_hw_teardown(c);
	/* do NOT wait for the last close: see clarett_hw_teardown() */
	snd_card_free_when_closed(card);
}

/*
 * remove() never runs on reboot/kexec; without this the device enters the next
 * kernel still bus-mastering with the old kernel's response buffer registered.
 */
static void clarett_shutdown(struct pci_dev *pci)
{
	struct snd_card *card = pci_get_drvdata(pci);
	struct clarett *c;

	if (!card)
		return;
	c = card->private_data;
	cancel_delayed_work_sync(&c->resume_work);
	cancel_delayed_work_sync(&c->follow_work);
	clarett_hwdep_free(c); /* no-op unless the hwdep path armed the relay */
	/*
	 * Persist a just-made change before the reboot tears the device down
	 * (mailbox still up).
	 */
	if (cancel_delayed_work_sync(&c->save_work))
		clarett_data_cmd(c, FCP_ACTIVATE_PERSIST);
	clarett_engine_stop(c);
	writel(0, c->bar0 + REG_IRQ0_ENABLE);
	clarett_quiesce_dma(pci, c->bar0);
}

static const struct pci_device_id clarett_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_FOCUSRITE, PCI_DEVICE_CLARETT),
	  .driver_data = (kernel_ulong_t)&clarett_2pre },
	{ }
};
MODULE_DEVICE_TABLE(pci, clarett_ids);

/*
 * The unit's serial, as the USB core exposes a USB device's:
 * /sys/bus/pci/devices/<bdf>/serial. Read from flash on the first read and
 * cached; one attempt, so a unit whose flash will not answer is not asked again
 * (-ENODATA thereafter). The PCI core creates the file after a successful probe
 * and removes it, waiting out any reader, before remove() runs, so drvdata (the
 * card) is valid whenever it is read.
 */
static ssize_t serial_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct clarett *c = card->private_data;
	int state;

	mutex_lock(&c->serial_lock);
	if (!c->serial_state) {
		clarett_read_serial(c);
		c->serial_state = c->serial[0] ? 1 : -1;
	}
	state = c->serial_state;
	mutex_unlock(&c->serial_lock);

	return state > 0 ? sysfs_emit(buf, "%s\n", c->serial) : -ENODATA;
}
static DEVICE_ATTR_RO(serial);

static struct attribute *clarett_attrs[] = {
	&dev_attr_serial.attr,
	NULL
};
ATTRIBUTE_GROUPS(clarett);

static struct pci_driver clarett_driver = {
	.name = KBUILD_MODNAME,
	/*
	 * Probe can spend up to CLARETT_READY_TIMEOUT_MS waiting for a device
	 * to become ready. Asynchronous so that wait runs on its own worker,
	 * not in line with every other driver's probe. (A removal of this
	 * device still waits for its probe to finish, which is why the budget
	 * is short.)
	 */
	.driver = {
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm = pm_sleep_ptr(&clarett_pm),
	},
	.id_table = clarett_ids,
	.probe = clarett_probe,
	.remove = clarett_remove,
	.dev_groups = clarett_groups,
	.shutdown = clarett_shutdown,
};
module_pci_driver(clarett_driver);

/*
 * Defined on the compiler command line by the Makefile, from dkms.conf. The
 * fallback is deliberately not a plausible number: it marks a build that came
 * in some other way, such as a bare kbuild invocation.
 */
#ifndef CLARETT_VERSION
#define CLARETT_VERSION "0.0.0-unknown"
#endif

MODULE_DESCRIPTION("Focusrite Clarett/Red (Thunderbolt) ALSA driver");
MODULE_AUTHOR("Miles Ramage <miles.ramage@yahoo.com>");
MODULE_VERSION(CLARETT_VERSION);
MODULE_LICENSE("GPL");
