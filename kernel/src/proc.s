; Pray that this works lmfao
extern REGS
extern lmao
extern determine_next_proc
extern PROC
global switch_ctx
global create_lmao

switch_ctx:
    ; Step 1: Push everything onto the motherfucking stack
    push rax
    push rbx
    push rcx
    push rdx
    push rsi
    push rdi
    push r8
    push r9
    push r10
    push r11
    push r12
    push r13
    push r14
    push r15

    ; Step 2: Put everything into storage
    mov rax, 14
.loop:
    dec rax
    pop rbx
    mov [REGS+rax*8], rbx
    cmp rax, 0
    jne .loop

    pop r8
    pop r9
    pop r10
    pop r11

    mov [PROC+8], r11
    mov [PROC], r10
    mov [PROC+16], r8

    push r11
    push r10
    push r9
    push r8

    ; Step 3: Determine which process we wanna execute next
    call determine_next_proc

    pop r8
    pop r9
    pop r10
    pop r11

    mov r11, [PROC+8]
    mov r10, [PROC]
    mov r8, [PROC+16]

    push r11
    push r10
    push r9
    push r8


    ; Step 4: Reenable lapic interrupt processing
    mov ecx, 0x80B
    mov eax, 0
    mov edx, 0
    wrmsr
    

    ; Step 5: Restore registers from the process
    mov rax, 14
.set_loop:
    dec rax
    push qword [REGS+rax*8]
    cmp rax, 0
    jne .set_loop

    pop r15
    pop r14
    pop r13
    pop r12
    pop r11
    pop r10
    pop r9
    pop r8
    pop rdi
    pop rsi
    pop rdx
    pop rcx
    pop rbx
    pop rax

    iretq

create_lmao:
    mov rsp, rdi
    jmp lmao
