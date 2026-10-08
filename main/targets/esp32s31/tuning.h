#pragma once
#include "rx_pll_cal.h"
#include "rx_recalibration.h"
/* Keep out-of-band requests out of the channel calibration/indexing path. */
static void s31_tune(unsigned mhz) {
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz-2412)%5 == 0) || mhz == 2484;
    phy_chip_set_chan(channel ? mhz : 2412, 0);
    phy_set_freq(mhz, 0);
    (void)rx_pll_recover(mhz);
}
