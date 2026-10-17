/* S31 SIMD core 1: per-bin power accumulation of an interleaved IQ16 FFT.
 * void s31_power_acc(const int16_t *x, uint64_t *acc, unsigned groups8)
 * acc[j] += re[j]^2 + im[j]^2 for 8 bins per group, in the eight 64-bit
 * lanes of QACC. The lanes are loaded from and stored back to acc, so the
 * accumulators persist across FFT blocks without any conversion; the
 * stored lane layout is plain little-endian uint64_t[8] per group.
 * Both pointers must be 16-byte aligned (x: HP SRAM). */
    .text
    .align  4
    .global s31_power_acc
    .type   s31_power_acc, @function
s31_power_acc:
    esp.lp.setup 0, a2, .Lacc_end
    esp.ld.qacc.l.l.128.ip a1, 16
    esp.ld.qacc.l.h.128.ip a1, 16
    esp.ld.qacc.h.l.128.ip a1, 16
    esp.ld.qacc.h.h.128.ip a1, -48
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a0, 16
    esp.vunzip.16 q0, q1          /* q0 = re0..re7, q1 = im0..im7 */
    esp.vmulas.s16.qacc q0, q0
    esp.vmulas.s16.qacc q1, q1
    esp.st.qacc.l.l.128.ip a1, 16
    esp.st.qacc.l.h.128.ip a1, 16
    esp.st.qacc.h.l.128.ip a1, 16
.Lacc_end: esp.st.qacc.h.h.128.ip a1, 16
    ret
    .size   s31_power_acc, . - s31_power_acc
