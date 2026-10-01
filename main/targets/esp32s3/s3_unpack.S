/*
 * ESP32-S3 PIE: IQ10 dump words -> windowed int16 FFT input.
 *
 *   void s3_unpack_iq10_win(const uint32_t *src, int16_t *dst,
 *                           const int16_t *win2, unsigned groups8);
 *
 * src     ring words (I bits 0..9, Q bits 10..19, signed), any 4-byte alignment
 * dst     16-byte aligned, receives {I<<6, Q<<6} * w >> 15 interleaved
 * win2    16-byte aligned window, each Q15 coefficient stored twice (I and Q)
 * groups8 number of 8-sample groups (samples / 8), >= 1
 *
 * Per 32-bit word: ((w << 6) & 0x0000ffc0) | ((w << 12) & 0xffc00000) puts
 * I<<6 into the low and Q<<6 into the high int16 lane; ee.vmul.s16 with
 * SAR = 15 then gives floor(I*w/512), bit-identical to the C reference
 * (I * w) >> 9. Unaligned source via ee.ld.128.usar.ip + ee.src.q; the
 * loads read up to 16 bytes past the last word (stays inside the bank).
 */
    .text
    .align  4
    .global s3_unpack_iq10_win
    .type   s3_unpack_iq10_win, @function
s3_unpack_iq10_win:
    entry   a1, 32
    movi    a6, 1023
    slli    a6, a6, 6               /* 0x0000ffc0 */
    slli    a7, a6, 16              /* 0xffc00000 */
    ee.movi.32.q q6, a6, 0
    ee.movi.32.q q6, a6, 1
    ee.movi.32.q q6, a6, 2
    ee.movi.32.q q6, a6, 3
    ee.movi.32.q q7, a7, 0
    ee.movi.32.q q7, a7, 1
    ee.movi.32.q q7, a7, 2
    ee.movi.32.q q7, a7, 3
    ee.ld.128.usar.ip q0, a2, 16
    loopnez a5, .Lunpack_end
    /* words 0..3 */
    ee.ld.128.usar.ip q1, a2, 16
    ee.src.q    q2, q0, q1
    ssai    6
    ee.vsl.32   q3, q2
    ssai    12
    ee.vsl.32   q4, q2
    ee.andq     q3, q3, q6
    ee.andq     q4, q4, q7
    ee.orq      q3, q3, q4
    ee.vld.128.ip q5, a4, 16
    ssai    15
    ee.vmul.s16 q3, q3, q5
    ee.vst.128.ip q3, a3, 16
    /* words 4..7 */
    ee.ld.128.usar.ip q0, a2, 16
    ee.src.q    q2, q1, q0
    ssai    6
    ee.vsl.32   q3, q2
    ssai    12
    ee.vsl.32   q4, q2
    ee.andq     q3, q3, q6
    ee.andq     q4, q4, q7
    ee.orq      q3, q3, q4
    ee.vld.128.ip q5, a4, 16
    ssai    15
    ee.vmul.s16 q3, q3, q5
    ee.vst.128.ip q3, a3, 16
.Lunpack_end:
    retw.n
    .size   s3_unpack_iq10_win, . - s3_unpack_iq10_win
