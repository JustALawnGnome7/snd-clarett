// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — system sleep (suspend and
 * resume).
 */
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <sound/asound.h>
#include <sound/core.h>
#include "clarett.h"

/*
 * System sleep (suspend to idle or RAM, and hibernation). The unit has its own
 * power supply and keeps its settings in flash, so nothing the user set needs
 * replaying; what has to be redone is the host's side of the session: the
 * response-buffer registration, the interrupt enable, the mailbox sequence, and
 * the stream engine, which the streams' re-prepare arms again.
 *
 * Suspend stops every mailbox user first, flushing a pending settings save
 * while the mailbox still works, then drops the device's DMA registration and
 * bus mastering as unload does. It must NOT save the PCI state itself: the PCI
 * core would then leave the device in D0, and with it the parent bridge (in the
 * unit's own Thunderbolt controller) powered while the chain above powers down,
 * which freezes a host behind a Thunderbolt dock on entry to suspend to idle.
 * Left to the core, the device goes to D3hot and keeps its state (it has
 * No_Soft_Reset), and the re-attach after resume redoes the host side.
 */
static int clarett_suspend(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	struct snd_card *card = pci_get_drvdata(pci);
	struct clarett *c;

	if (!card)
		return 0;
	c = card->private_data;

	/* a re-attach still pending from the last resume */
	cancel_delayed_work_sync(&c->resume_work);
	/* new hwdep/control calls wait */
	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);
	/* and the ones in flight finish */
	snd_power_sync_ref(card);

	WRITE_ONCE(c->ctl_ready, false); /* no new notify work from the ISR */
	cancel_delayed_work_sync(&c->meter_work);
	cancel_delayed_work_sync(&c->follow_work);
	cancel_work_sync(&c->notify_work);
	if (cancel_delayed_work_sync(&c->save_work))
		clarett_data_cmd(c, FCP_ACTIVATE_PERSIST);
	clarett_midi_suspend(c);
	clarett_pcm_suspend(c);

	writel(0, c->bar0 + REG_IRQ0_ENABLE);
	clarett_quiesce_dma(pci, c->bar0);
	return 0;
}

/*
 * Resume runs from a work item a moment after the host is awake, not in the
 * resume callback, which runs while the Thunderbolt link to the unit is still
 * being re-established. The card stays at D3hot until the work finishes, so
 * fcp-server's ioctls and the PCM opens wait for it.
 *
 * The work repeats the attach (clarett_hw_init(), which waits for the device to
 * acknowledge the response address) with the mailbox sequence back at 0, then
 * re-seeds the shadow, restores the monitor enables and the Clock Source, and
 * tells fcp-server to re-read every control, since the front panel may have
 * moved during the sleep. A unit that went away meanwhile (its config space no
 * longer answers) is left for the hotplug removal that follows.
 */
#define CLARETT_RESUME_DELAY_MS	1000

void clarett_resume_work(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett,
					 resume_work.work);
	struct pci_dev *pci = c->pci;
	int err;

	if (!pci_device_is_present(pci) || readl(c->bar0 + REG_CAPS) == ~0u) {
		dev_warn(&pci->dev,
			 "resume: device not reachable; leaving it to be removed\n");
		goto out;
	}

	pci_set_master(pci);
	c->seq = 0;
	c->lost_pending = false;
	c->mbox_wedged = false;
	c->mon_snap_valid = false;
	clarett_hw_init(c);

	err = clarett_seed_shadow(c);
	if (err) {
		dev_warn(&pci->dev, "resume: config shadow seed failed (%d)\n",
			 err);
	} else {
		err = clarett_enable_monitor_hw_controls(c);
		if (err)
			dev_warn(&pci->dev,
				 "resume: could not enable monitor hardware mute/dim (%d)\n",
				 err);
	}
	clarett_pcm_resume(c);

	clarett_meter_poll_start(c);
	WRITE_ONCE(c->ctl_ready, true);
	clarett_schedule_follow(c);
	clarett_hwdep_notify(c, ~0u);
	clarett_midi_resume(c);
out:
	snd_power_change_state(c->card, SNDRV_CTL_POWER_D0);
}

static int clarett_resume(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct clarett *c;

	if (!card)
		return 0;
	c = card->private_data;
	schedule_delayed_work(&c->resume_work,
			      msecs_to_jiffies(CLARETT_RESUME_DELAY_MS));
	return 0;
}

DEFINE_SIMPLE_DEV_PM_OPS(clarett_pm, clarett_suspend, clarett_resume);
