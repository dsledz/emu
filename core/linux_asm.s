.globl SwitchContext
.globl InitialSwitchContext

InitialSwitchContext:
    /* Store the old registers */
    mov %r8,   0(%rdi)
    mov %r9,   8(%rdi)
    mov %r10, 16(%rdi)
    mov %r11, 24(%rdi)
    mov %r12, 32(%rdi)
    mov %r13, 40(%rdi)
    mov %r14, 48(%rdi)
    mov %r15, 56(%rdi)
    mov %rsp, 64(%rdi)
    mov %rbp, 72(%rdi)
    mov %rbx, 80(%rdi)

    /* Restore the current registers */
    mov  0(%rsi), %r8
    mov  8(%rsi), %r9
    mov 16(%rsi), %r10
    mov 24(%rsi), %r11
    mov 32(%rsi), %r12
    mov 40(%rsi), %r13
    mov 48(%rsi), %r14
    mov 56(%rsi), %r15
    mov 64(%rsi), %rsp
    mov 72(%rsi), %rbp
    mov 80(%rsi), %rbx

    mov %rdx, %rdi

    ret

SwitchContext:
    /* Store the old registers */
    mov %r8,   0(%rdi)
    mov %r9,   8(%rdi)
    mov %r10, 16(%rdi)
    mov %r11, 24(%rdi)
    mov %r12, 32(%rdi)
    mov %r13, 40(%rdi)
    mov %r14, 48(%rdi)
    mov %r15, 56(%rdi)
    mov %rsp, 64(%rdi)
    mov %rbp, 72(%rdi)
    mov %rbx, 80(%rdi)

    /* Restore the current registers */
    mov  0(%rsi), %r8
    mov  8(%rsi), %r9
    mov 16(%rsi), %r10
    mov 24(%rsi), %r11
    mov 32(%rsi), %r12
    mov 40(%rsi), %r13
    mov 48(%rsi), %r14
    mov 56(%rsi), %r15
    mov 64(%rsi), %rsp
    mov 72(%rsi), %rbp
    mov 80(%rsi), %rbx

    ret
