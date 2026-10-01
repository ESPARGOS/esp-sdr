/*
 * ESP32-S3 app CPU (core 1) entry for the SPEC worker, used while ESP-IDF
 * runs single-core (CONFIG_FREERTOS_UNICORE): IDF leaves core 1 stalled, and
 * s3_ring.c releases it here with ets_set_appcpu_boot_addr().
 *
 * Core 1 runs with all interrupts masked (PS.INTLEVEL 15), windowed ABI with
 * window-overflow exceptions through IDF's vector table, FPU (CP0) and PIE
 * (CP3) enabled, and its own 8 KiB stack. It never returns. Setup sequence
 * after h0m3us3r/eSpDR startup.S (0BSD).
 */
    .section .iram1.s3_core1, "ax"
    .literal_position
    .align  4
    .global s3_core1_entry
    .type   s3_core1_entry, @function
s3_core1_entry:
    movi    a2, 0
    wsr     a2, intenable
    wsr     a2, windowbase
    movi    a2, 1
    wsr     a2, windowstart
    movi    a2, 9                   /* CP0 (FPU) + CP3 (PIE) */
    wsr     a2, cpenable
    movi    a2, _vector_table
    wsr     a2, vecbase
    movi    a2, 0x4002f             /* WOE, UM, INTLEVEL 15 */
    wsr     a2, ps
    rsync
    movi    a1, s3_core1_stack_top
    mov     a2, a1
    addi    a1, a1, -32
    s32e    a2, a1, -12
    movi    a0, 0
    call8   s3_core1_main
1:  j       1b
    .size   s3_core1_entry, . - s3_core1_entry

    .section .bss.s3_core1_stack, "aw", @nobits
    .align  16
    .global s3_core1_stack
s3_core1_stack:
    .space  8192
    .global s3_core1_stack_top
s3_core1_stack_top:
