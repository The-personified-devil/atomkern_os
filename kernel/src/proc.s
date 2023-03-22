extern lmao
extern determine_next_proc

global switch_ctx
global create_lmao

switch_ctx:
    ; Step 1: Push everything onto the stack
    ; Treat as array, i.e. class MEMORY, so on stack with sysv-abi
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

    ; Step 3: Determine which process we wanna execute next
    call determine_next_proc

    ; Clear all previous data from stack
    add rsp, 19 * 8

    ; First things is ISR stack data, then regs from r15 to rax
    ; Rax contains returned value, cuz sysv-abi
    mov rcx, 0
.set_loop:
    push qword [rax+rcx*8]
    inc rcx
    cmp rcx, 19
    jne .set_loop

    ; Step 4: Reenable lapic interrupt processing
    ; Do here to not mess with rax from sysv-abi and also as late as possible to not cause int trouble
    mov ecx, 0x80B
    mov eax, 0
    mov edx, 0
    wrmsr

    ; Rax pushed onto stack last, so pop first
    pop rax
    pop rbx
    pop rcx
    pop rdx
    pop rsi
    pop rdi
    pop r8
    pop r9
    pop r10
    pop r11
    pop r12
    pop r13
    pop r14
    pop r15

    ; mov fs, r15
    mov r15, [rax+20*8]
    ; mov cr3, r15
    mov r15,fs

    iretq

create_lmao:
    mov rsp, rdi
    jmp lmao
