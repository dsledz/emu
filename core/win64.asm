.CODE

InitialSwitchContext PROC
    mov [rdi], r8
    mov [rdi + 8], r9
    mov [rdi + 16], r10
    mov [rdi + 24], r11
    mov [rdi + 32], r12
    mov [rdi + 40], r13
    mov [rdi + 48], r14
    mov [rdi + 56], r15
    mov [rdi + 64], rsp
    mov [rdi + 72], rbp
    mov [rdi + 80], rbx

    mov r8,  [rsi]
    mov r9,  [rsi + 8]
    mov r10, [rsi + 16]
    mov r11, [rsi + 24]
    mov r12, [rsi + 32]
    mov r13, [rsi + 40]
    mov r14, [rsi + 48]
    mov r15, [rsi + 56]
    mov rsp, [rsi + 64]
    mov rbp, [rsi + 72]
    mov rbx, [rsi + 80]

    mov rdi, rdx

    ret
InitialSwitchContext ENDP

SwitchContext PROC
    mov [rdi], r8
    mov [rdi + 8], r9
    mov [rdi + 16], r10
    mov [rdi + 24], r11
    mov [rdi + 32], r12
    mov [rdi + 40], r13
    mov [rdi + 48], r14
    mov [rdi + 56], r15
    mov [rdi + 64], rsp
    mov [rdi + 72], rbp
    mov [rdi + 80], rbx

    mov r8,  [rsi]
    mov r9,  [rsi + 8]
    mov r10, [rsi + 16]
    mov r11, [rsi + 24]
    mov r12, [rsi + 32]
    mov r13, [rsi + 40]
    mov r14, [rsi + 48]
    mov r15, [rsi + 56]
    mov rsp, [rsi + 64]
    mov rbp, [rsi + 72]
    mov rbx, [rsi + 80]

    ret
SwitchContext ENDP

END
