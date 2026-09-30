// Radix-2 complex int16 FFT for the ESP32-S3 SPEC mode, derived from esp-dsp
// dsps_fft2r_sc16_aes3_ (Copyright 2018-2019 Espressif Systems (Shanghai) PTE LTD,
// Apache License 2.0). Same data layout, twiddle table and bit-reversed output.
//
// Change: esp-dsp's aes3 kernel floors a*32767>>16 and the twiddle product >>16
// separately, so every butterfly's sum output is biased by about -1 LSB while the
// difference output is not. On weak signals (noise of a few LSB) this bias shows up
// as a one-sided noise ramp with a step at 0 Hz (up to ~15 dB at 2048 points).
// Adding +1 to the sum output of every butterfly (one ee.vadds per vector) removes
// the mean bias; the ANSI reference (which rounds) and a float FFT agree to ~1 dB.
//
// int s3_fft2r_sc16_rnd(int16_t *data, int N, int16_t *w)   N power of two, >= 8
    .text
    .align  4
    .global s3_fft2r_sc16_rnd
    .type   s3_fft2r_sc16_rnd, @function
s3_fft2r_sc16_rnd:
    entry   a1, 64
    mov.n   a9, a1
    movi.n  a8, 1
    slli    a8, a8, 15
    addi.n  a8, a8, -1              // 32767
    addi.n  a6, a3, 1
    movi.n  a10, 16
    wsr.sar a10
    movgez  a6, a3, a3
    s16i    a8, a1, 0               // temp_round = 32767
    ee.vldbc.16.ip  q6, a9, 0       // q6 = 32767 x8 (the /2 of every stage)
    movi.n  a11, 1
    s16i    a11, a1, 2
    addi.n  a11, a1, 2
    ee.vldbc.16.ip  q7, a11, 0      // q7 = 1 x8 (bias correction)
    srai    a6, a6, 1
    bltui   a6, 3, .Lr_short

    movi.n  a5, 1
.Lr_stage:
    mov.n   a13, a4
    mov.n   a9, a2
    beqz.n  a5, .Lr_next
    srli    a3, a6, 2
    movi.n  a14, 0
    slli    a15, a6, 2
    add.n   a8, a15, a9
.Lr_group:
    ee.vldbc.32.ip  q5, a13, 4
    loopnez a3, .Lr_group_end
    ee.vld.128.ip   q0, a8, 0
    ee.vld.128.ip   q2, a9, 0
    ee.cmul.s16     q1, q5, q0, 2
    ee.vmul.s16     q2, q2, q6
    ee.cmul.s16     q1, q5, q0, 3
    ee.vsubs.s16    q3, q2, q1
    ee.vadds.s16.st.incp q3, a8, q3, q2, q1
    ee.vadds.s16    q3, q3, q7      // sum + 1
    ee.vst.128.ip   q3, a9, 16
.Lr_group_end:
    addi.n  a14, a14, 1
    add.n   a9, a9, a15
    add.n   a8, a15, a9
    bne     a14, a5, .Lr_group
.Lr_next:
    slli    a5, a5, 1
    srli    a6, a6, 1
    bgeui   a6, 3, .Lr_stage

    srli    a10, a5, 1
    beqz.n  a10, .Lr_skip2
    mov.n   a9, a4
    mv.qr   q4, q1
    mov.n   a8, a2
    mv.qr   q1, q0
    mv.qr   q0, q2
    loopnez a10, .Lr_s2_end
    ee.vld.l.64.ip  q0, a8, 8
    ee.vldbc.32.ip  q2, a9, 4
    ee.vld.l.64.ip  q1, a8, 8
    ee.vldbc.32.ip  q3, a9, 4
    ee.vld.h.64.ip  q0, a8, 8
    ee.vunzip.32    q2, q3
    ee.vld.h.64.ip  q1, a8, -24
    ee.vmul.s16     q0, q0, q6
    ee.cmul.s16     q4, q2, q1, 2
    ee.cmul.s16     q4, q2, q1, 3
    ee.vadds.s16    q2, q0, q4
    ee.vsubs.s16    q3, q0, q4
    ee.vadds.s16    q2, q2, q7      // sum + 1
    ee.vst.l.64.ip  q2, a8, 8
    ee.vst.l.64.ip  q3, a8, 8
    ee.vst.h.64.ip  q2, a8, 8
    ee.vst.h.64.ip  q3, a8, 8
.Lr_s2_end:
    st.qr   q4, a1, 16
.Lr_last:
    ld.qr   q3, a1, 16
    slli    a10, a5, 1
    srli    a10, a10, 2
    loopnez a10, .Lr_s1_end
    ee.vld.128.ip   q0, a2, 16
    ee.vld.128.ip   q1, a2, -16
    ee.vld.128.ip   q2, a4, 16
    ee.vunzip.32    q0, q1
    ee.cmul.s16     q3, q2, q1, 2
    ee.vmul.s16     q0, q0, q6
    ee.cmul.s16     q3, q2, q1, 3
    ee.vsubs.s16    q1, q0, q3
    ee.vadds.s16    q0, q0, q3
    ee.vadds.s16    q0, q0, q7      // sum + 1
    ee.vzip.32      q0, q1
    ee.vst.128.ip   q0, a2, 16
    ee.vst.128.ip   q1, a2, 16
.Lr_s1_end:
    movi.n  a2, 0
    retw.n
.Lr_short:
    movi.n  a5, 1
    j       .Lr_last
.Lr_skip2:
    st.qr   q1, a1, 16
    j       .Lr_last
    .size   s3_fft2r_sc16_rnd, . - s3_fft2r_sc16_rnd
