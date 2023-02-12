; this code will be relocated to 0x8000, sets up environment for calling a C function
[BITS 16]
ap_trampoline:
    cli
    cld
    jmp    0:0x8040

align 16
_L8010_GDT_table:
.Data:
    dd 0, 0
    dd 0x0000FFFF, 0x00CF9A00    ; flat code
    dd 0x0000FFFF, 0x008F9200    ; flat data
    dd 0x00000068, 0x00CF8900    ; tss
_L8030_GDT_value:
    db _L8030_GDT_value - _L8010_GDT_table - 1
    dd 0x8010
    dd 0, 0
    align 64
_L8040:
    xor    ax, ax
    mov    ds, ax
    lgdt   [0x8030]
    mov     eax, cr0
    or     eax, 1
    mov    cr0, eax
    jmp    8:0x8060

align 32
[BITS 32]
_L8060:
    mov    ax, 16 
    mov    ds, ax
    mov     ss, ax
    ; get our Local APIC ID
    mov     eax, 1
    cpuid
    shr    ebx, 24
    mov    edi, ebx 
    ; set up 32k stack, one for each core. It is important that all core must have its own stack
    shl    ebx, 15
    ; mov    esp, stack_top
    sub    esp, ebx
    ; push   edi
    ; spinlock, wait for the BSP to finish
Loop:
    jmp Loop
; 1:  pause
;     cmpb    $0, bspdone
;     jz      1b
;     lock    incb aprunning
;     ; jump into C code (should never return)
;     ljmp    $8, $ap_startup
