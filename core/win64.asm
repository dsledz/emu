.CODE

InitialSwitchContext PROC
    mov [rcx], rbx
    mov [rcx + 8], rdi
    mov [rcx + 16], rsi
    mov [rcx + 24], r12
    mov [rcx + 32], r13
    mov [rcx + 40], r14
    mov [rcx + 48], r15
    mov [rcx + 56], rsp
    mov [rcx + 64], rbp

    mov rbx,  [rdx]
    mov rdi,  [rdx + 8]
    mov rsi, [rdx + 16]
    mov r12, [rdx + 24]
    mov r13, [rdx + 32]
    mov r14, [rdx + 40]
    mov r15, [rdx + 48]
    mov rsp, [rdx + 56]
    mov rbp, [rdx + 64]

    mov rcx, r8

    ret
InitialSwitchContext ENDP

SwitchContext PROC
   mov [rcx], rbx
mov [rcx + 8], rdi
mov [rcx + 16], rsi
mov [rcx + 24], r12
mov [rcx + 32], r13
mov [rcx + 40], r14
mov [rcx + 48], r15
mov [rcx + 56], rsp
mov [rcx + 64], rbp

mov rbx,  [rdx]
mov rdi,  [rdx + 8]
mov rsi, [rdx + 16]
mov r12, [rdx + 24]
mov r13, [rdx + 32]
mov r14, [rdx + 40]
mov r15, [rdx + 48]
mov rsp, [rdx + 56]
mov rbp, [rdx + 64]

    ret
SwitchContext ENDP

END
