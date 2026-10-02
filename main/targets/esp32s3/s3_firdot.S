/*
 * ESP32-S3 PIE kernels for the IQS decimating FIR (split I / Q arrays).
 *
 * void s3_unpack_iq10_split(const uint32_t *src, int16_t *di, int16_t *dq, unsigned groups8);
 *   src: ring words (I bits 0..9, Q bits 10..19), any 4-byte alignment; reads up
 *        to 16 bytes past the last word. di, dq: 16-byte aligned. Writes
 *        I << 6 and Q << 6 as int16, 8 samples per group.
 *
 * void s3_fir_split(const fir_job_t *job);
 *   job: +0 xi, +4 xq (16-byte aligned window starts), +8 h (16-byte aligned,
 *        taps in window order), +12 groups (taps / 8), +16 nout, +20 step
 *        (bytes between successive windows, multiple of 16), +24 shift,
 *        +28 oi, +32 oq (int16 outputs, any alignment).
 *   For each output: oi[k] = (sum h[j] * xi[k*step/2 + j]) >> shift, same for Q.
 */
    .section .iram1.s3_firdot, "ax"
    .align  4
    .global s3_unpack_iq10_split
    .type   s3_unpack_iq10_split, @function
s3_unpack_iq10_split:
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
    loopnez a5, .Lsplit_end
    /* words 0..3 -> q3 = {I0,Q0,..,I3,Q3} << 6 */
    ee.ld.128.usar.ip q1, a2, 16
    ee.src.q    q2, q0, q1
    ssai    6
    ee.vsl.32   q3, q2
    ssai    12
    ee.vsl.32   q4, q2
    ee.andq     q3, q3, q6
    ee.andq     q4, q4, q7
    ee.orq      q3, q3, q4
    /* words 4..7 -> q5 */
    ee.ld.128.usar.ip q0, a2, 16
    ee.src.q    q2, q1, q0
    ssai    6
    ee.vsl.32   q5, q2
    ssai    12
    ee.vsl.32   q4, q2
    ee.andq     q5, q5, q6
    ee.andq     q4, q4, q7
    ee.orq      q5, q5, q4
    /* even lanes (I) -> q3, odd lanes (Q) -> q5 */
    ee.vunzip.16 q3, q5
    ee.vst.128.ip q3, a3, 16
    ee.vst.128.ip q5, a4, 16
.Lsplit_end:
    retw.n
    .size   s3_unpack_iq10_split, . - s3_unpack_iq10_split

    .align  4
    .global s3_fir_split
    .type   s3_fir_split, @function
s3_fir_split:
    entry   a1, 32
    l32i    a3, a2, 16              /* nout */
    beqz    a3, .Lfs_done
    l32i    a4, a2, 0               /* xi */
    l32i    a5, a2, 4               /* xq */
    l32i    a6, a2, 28              /* oi */
    l32i    a7, a2, 32              /* oq */
    l32i    a10, a2, 12             /* groups */
    l32i    a11, a2, 24             /* shift */
    l32i    a13, a2, 20             /* step */
.Lfs_out:
    l32i    a8, a2, 8               /* h */
    mov     a9, a4
    ee.zero.accx
    loopnez a10, .Lfs_dot_i
    ee.vld.128.ip q0, a8, 16
    ee.vld.128.ip q1, a9, 16
    ee.vmulas.s16.accx q0, q1
.Lfs_dot_i:
    ee.srs.accx a12, a11, 0
    s16i    a12, a6, 0
    addi    a6, a6, 2
    l32i    a8, a2, 8
    mov     a9, a5
    ee.zero.accx
    loopnez a10, .Lfs_dot_q
    ee.vld.128.ip q0, a8, 16
    ee.vld.128.ip q1, a9, 16
    ee.vmulas.s16.accx q0, q1
.Lfs_dot_q:
    ee.srs.accx a12, a11, 0
    s16i    a12, a7, 0
    addi    a7, a7, 2
    add     a4, a4, a13
    add     a5, a5, a13
    addi    a3, a3, -1
    bnez    a3, .Lfs_out
.Lfs_done:
    retw.n
    .size   s3_fir_split, . - s3_fir_split

/*
 * void s3_fir8_l32(const fir_job_t *job): stage-1 special case, 32 taps,
 * step 8 samples (16 bytes). Taps live in q4..q7 for the whole call; each
 * window is 4 vectors, loads fused into the multiply-accumulates.
 */
    .align  4
    .global s3_fir8_l32
    .type   s3_fir8_l32, @function
s3_fir8_l32:
    entry   a1, 32
    l32i    a3, a2, 16              /* nout */
    beqz    a3, .Lf8_done
    l32i    a4, a2, 0               /* xi */
    l32i    a5, a2, 4               /* xq */
    l32i    a8, a2, 8               /* h */
    l32i    a6, a2, 28              /* oi */
    l32i    a7, a2, 32              /* oq */
    l32i    a11, a2, 24             /* shift */
    ee.vld.128.ip q4, a8, 16
    ee.vld.128.ip q5, a8, 16
    ee.vld.128.ip q6, a8, 16
    ee.vld.128.ip q7, a8, 16
    loopnez a3, .Lf8_end
    mov     a9, a4
    ee.zero.accx
    ee.vld.128.ip q0, a9, 16
    ee.vmulas.s16.accx.ld.ip q1, a9, 16, q0, q4
    ee.vmulas.s16.accx.ld.ip q0, a9, 16, q1, q5
    ee.vmulas.s16.accx.ld.ip q1, a9, 16, q0, q6
    ee.vmulas.s16.accx q1, q7
    ee.srs.accx a12, a11, 0
    s16i    a12, a6, 0
    mov     a9, a5
    ee.zero.accx
    ee.vld.128.ip q0, a9, 16
    ee.vmulas.s16.accx.ld.ip q1, a9, 16, q0, q4
    ee.vmulas.s16.accx.ld.ip q0, a9, 16, q1, q5
    ee.vmulas.s16.accx.ld.ip q1, a9, 16, q0, q6
    ee.vmulas.s16.accx q1, q7
    ee.srs.accx a12, a11, 0
    s16i    a12, a7, 0
    addi    a6, a6, 2
    addi    a7, a7, 2
    addi    a4, a4, 16
    addi    a5, a5, 16
.Lf8_end:
.Lf8_done:
    retw.n
    .size   s3_fir8_l32, . - s3_fir8_l32


/*
 * void s3_rot_fs4(int16_t *wi, int16_t *wq, unsigned groups8, const int16_t *masks);
 * In place: z[n] *= j^n (shift by +fs/4), n = 0 at wi[0] (callers keep wi 8-aligned
 * to the stream index). masks: m1 = {1,0,-1,0,..}, m2 = {0,-1,0,1,..}, m3 = -m2,
 * 16 bytes each, 16-byte aligned.  I' = I*m1 + Q*m2, Q' = Q*m1 + I*m3.
 */
    .align  4
    .global s3_rot_fs4
    .type   s3_rot_fs4, @function
s3_rot_fs4:
    entry   a1, 32
    ee.vld.128.ip q4, a5, 16
    ee.vld.128.ip q5, a5, 16
    ee.vld.128.ip q6, a5, 16
    ssai    0
    mov     a6, a2                  /* store pointers */
    mov     a7, a3
    loopnez a4, .Lrot_end
    ee.vld.128.ip q0, a2, 16        /* I */
    ee.vld.128.ip q1, a3, 16        /* Q */
    ee.vmul.s16 q2, q0, q4
    ee.vmul.s16 q3, q1, q5
    ee.vadds.s16 q2, q2, q3
    ee.vmul.s16 q3, q1, q4
    ee.vmul.s16 q7, q0, q6
    ee.vadds.s16 q3, q3, q7
    ee.vst.128.ip q2, a6, 16
    ee.vst.128.ip q3, a7, 16
.Lrot_end:
    retw.n
    .size   s3_rot_fs4, . - s3_rot_fs4
