#pragma once

/* Receiver and capture must be stopped. Measure fresh RX DC at mhz and rebuild
 * the gain tables while retaining the IQ correction established at startup.
 * Retuning must not start a transmitting loopback IQ calibration. Caller restores
 * channel, filter, RX power and gain settings afterwards. Stock startup and
 * background PHY calibration remain outside this receive-only retune path. */
void rx_recalibrate(unsigned mhz);

/* H2 only: call once during boot, before exposing receive commands. Its PHY
 * needs separate startup IQ corrections for each RF gain group. */
void rx_h2_calibrate_iq_at_boot(void);
