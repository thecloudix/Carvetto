	.file	"robust_predicates.cpp"
	.text
	.section	.rodata.str1.1,"aMS",@progbits,1
.LC0:
	.string	"vector::reserve"
.LC1:
	.string	"vector::_M_realloc_append"
	.section	.text.unlikely,"ax",@progbits
.LCOLDB4:
	.text
.LHOTB4:
	.p2align 4
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_:
.LFB2287:
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDA2287
	pushq	%r15
	.cfi_def_cfa_offset 16
	.cfi_offset 15, -16
	pushq	%r14
	.cfi_def_cfa_offset 24
	.cfi_offset 14, -24
	pushq	%r13
	.cfi_def_cfa_offset 32
	.cfi_offset 13, -32
	pushq	%r12
	.cfi_def_cfa_offset 40
	.cfi_offset 12, -40
	pushq	%rbp
	.cfi_def_cfa_offset 48
	.cfi_offset 6, -48
	pushq	%rbx
	.cfi_def_cfa_offset 56
	.cfi_offset 3, -56
	subq	$72, %rsp
	.cfi_def_cfa_offset 128
	movq	8(%rsi), %r8
	movq	(%rsi), %r9
	movq	%rdi, 8(%rsp)
	cmpq	%r8, %r9
	je	.L148
	movq	8(%rdx), %r13
	movq	(%rdx), %r10
	subq	%r9, %r8
	cmpq	%r13, %r10
	je	.L149
	subq	%r10, %r13
	sarq	$3, %r8
	movabsq	$1152921504606846975, %r14
	sarq	$3, %r13
	leaq	(%r8,%r13), %rbp
	cmpq	%rbp, %r14
	jb	.L131
	testq	%rbp, %rbp
	je	.L67
	salq	$3, %rbp
	movq	%r10, 16(%rsp)
	movq	%rbp, %rdi
	movq	%r9, 24(%rsp)
	movq	%r8, 32(%rsp)
.LEHB0:
	call	_Znwm@PLT
.LEHE0:
	movq	16(%rsp), %r10
	movq	24(%rsp), %r9
	addq	%rax, %rbp
	testq	%r13, %r13
	movq	32(%rsp), %r8
	je	.L12
	testq	%r8, %r8
	je	.L137
	movq	%rax, %r12
	xorl	%r15d, %r15d
	xorl	%ebx, %ebx
	jmp	.L10
	.p2align 4,,10
	.p2align 3
.L28:
	incq	%rbx
	cmpq	%rbp, %rax
	je	.L13
	vmovsd	%xmm1, (%rax)
	addq	$8, %rax
.L14:
	cmpq	%r8, %rbx
	jnb	.L27
.L151:
	cmpq	%r13, %r15
	jnb	.L150
.L10:
	vmovsd	(%r9,%rbx,8), %xmm1
	vmovsd	(%r10,%r15,8), %xmm0
	vandpd	.LC2(%rip), %xmm1, %xmm2
	vandpd	.LC2(%rip), %xmm0, %xmm3
	vcomisd	%xmm2, %xmm3
	ja	.L28
	incq	%r15
	cmpq	%rbp, %rax
	je	.L18
	vmovsd	%xmm0, (%rax)
	addq	$8, %rax
	cmpq	%r8, %rbx
	jb	.L151
	.p2align 4,,10
	.p2align 3
.L27:
	cmpq	%r13, %r15
	jb	.L42
	jmp	.L36
	.p2align 5
	.p2align 4,,10
	.p2align 3
.L152:
	vmovsd	(%r10,%r15,8), %xmm0
	incq	%r15
	addq	$8, %rax
	vmovsd	%xmm0, -8(%rax)
	cmpq	%r15, %r13
	je	.L36
.L42:
	cmpq	%rbp, %rax
	jne	.L152
	movq	%rbp, %rdx
	movabsq	$1152921504606846975, %rdi
	subq	%r12, %rdx
	movq	%rdx, %rax
	sarq	$3, %rax
	cmpq	%rdi, %rax
	je	.L135
	testq	%rax, %rax
	movl	$1, %ebx
	movq	%rdx, 24(%rsp)
	cmovne	%rax, %rbx
	movq	%r10, 16(%rsp)
	addq	%rax, %rbx
	movabsq	$1152921504606846975, %rax
	cmpq	%rax, %rbx
	cmova	%rax, %rbx
	salq	$3, %rbx
	movq	%rbx, %rdi
.LEHB1:
	call	_Znwm@PLT
	movq	16(%rsp), %r10
	movq	24(%rsp), %rdx
	movq	%rax, %r14
	vmovsd	(%r10,%r15,8), %xmm0
	vmovsd	%xmm0, (%rax,%rdx)
	testq	%rdx, %rdx
	je	.L40
	movq	%r12, %rsi
	movq	%rax, %rdi
	movq	%r10, 24(%rsp)
	movq	%rdx, 16(%rsp)
	call	memcpy@PLT
	movq	24(%rsp), %r10
	movq	16(%rsp), %rdx
.L40:
	leaq	8(%r14,%rdx), %rax
	testq	%r12, %r12
	je	.L41
	movq	%rbp, %rsi
	movq	%r12, %rdi
	movq	%r10, 24(%rsp)
	subq	%r12, %rsi
	movq	%rax, 16(%rsp)
	call	_ZdlPvm@PLT
	movq	24(%rsp), %r10
	movq	16(%rsp), %rax
.L41:
	incq	%r15
	leaq	(%r14,%rbx), %rbp
	movq	%r14, %r12
	cmpq	%r15, %r13
	jne	.L42
	.p2align 4,,10
	.p2align 3
.L36:
	subq	%r12, %rax
	movq	%rax, %r13
	je	.L9
	movq	%rax, %rdi
	call	_Znwm@PLT
.LEHE1:
	vmovsd	(%r12), %xmm1
	leaq	(%rax,%r13), %rbx
	cmpq	$8, %r13
	jle	.L71
	movq	%r13, %rcx
	movq	%rax, %r14
	movl	$1, %r15d
	sarq	$3, %rcx
	.p2align 4,,10
	.p2align 3
.L52:
	vmovsd	(%r12,%r15,8), %xmm2
	vmovapd	%xmm1, %xmm0
	vaddsd	%xmm2, %xmm1, %xmm1
	vsubsd	%xmm0, %xmm1, %xmm3
	vsubsd	%xmm3, %xmm1, %xmm4
	vsubsd	%xmm3, %xmm2, %xmm2
	vsubsd	%xmm4, %xmm0, %xmm0
	vaddsd	%xmm2, %xmm0, %xmm0
	vxorpd	%xmm2, %xmm2, %xmm2
	vucomisd	%xmm2, %xmm0
	jp	.L75
	je	.L46
.L75:
	cmpq	%rax, %rbx
	je	.L48
	vmovsd	%xmm0, (%rax)
	addq	$8, %rax
.L46:
	incq	%r15
	cmpq	%rcx, %r15
	jb	.L52
	vxorpd	%xmm0, %xmm0, %xmm0
	vucomisd	%xmm0, %xmm1
	jp	.L53
	je	.L153
.L53:
	cmpq	%rax, %rbx
	je	.L154
.L45:
	vmovsd	%xmm1, (%rax)
	addq	$8, %rax
.L56:
	vmovq	%r14, %xmm5
	movq	%rbp, %rsi
	movq	%r12, %rdi
	vpinsrq	$1, %rax, %xmm5, %xmm0
	movq	8(%rsp), %rax
	subq	%r12, %rsi
	movq	%rbx, 16(%rax)
	vmovdqu	%xmm0, (%rax)
	call	_ZdlPvm@PLT
.L1:
	movq	8(%rsp), %rax
	addq	$72, %rsp
	.cfi_remember_state
	.cfi_def_cfa_offset 56
	popq	%rbx
	.cfi_def_cfa_offset 48
	popq	%rbp
	.cfi_def_cfa_offset 40
	popq	%r12
	.cfi_def_cfa_offset 32
	popq	%r13
	.cfi_def_cfa_offset 24
	popq	%r14
	.cfi_def_cfa_offset 16
	popq	%r15
	.cfi_def_cfa_offset 8
	ret
	.p2align 4,,10
	.p2align 3
.L67:
	.cfi_restore_state
	xorl	%ebp, %ebp
	xorl	%r12d, %r12d
.L9:
	vmovsd	(%r12), %xmm1
	xorl	%r13d, %r13d
	xorl	%ebx, %ebx
	xorl	%ecx, %ecx
	xorl	%r15d, %r15d
.L44:
	testq	%rcx, %rcx
	movl	$1, %eax
	vmovsd	%xmm1, 24(%rsp)
	cmovne	%rcx, %rax
	addq	%rcx, %rax
	movabsq	$1152921504606846975, %rcx
	cmpq	%rcx, %rax
	cmova	%rcx, %rax
	salq	$3, %rax
	movq	%rax, %rdi
	movq	%rax, 16(%rsp)
.LEHB2:
	call	_Znwm@PLT
.LEHE2:
	vmovsd	24(%rsp), %xmm1
	movq	%rax, %r14
	vmovsd	%xmm1, (%rax,%r15)
	testq	%r15, %r15
	jne	.L155
.L58:
	leaq	8(%r14,%r15), %rax
	testq	%r13, %r13
	je	.L59
	movq	%rbx, %rsi
	movq	%r13, %rdi
	movq	%rax, 24(%rsp)
	subq	%r13, %rsi
	call	_ZdlPvm@PLT
	movq	24(%rsp), %rax
.L59:
	movq	16(%rsp), %rbx
	addq	%r14, %rbx
	jmp	.L56
	.p2align 4,,10
	.p2align 3
.L13:
	movq	%rbp, %rdx
	subq	%r12, %rdx
	movq	%rdx, %rsi
	sarq	$3, %rsi
	cmpq	%r14, %rsi
	je	.L132
	testq	%rsi, %rsi
	movl	$1, %eax
	movq	%r8, 56(%rsp)
	cmovne	%rsi, %rax
	movq	%rdx, 48(%rsp)
	movq	%r9, 40(%rsp)
	addq	%rsi, %rax
	movabsq	$1152921504606846975, %rsi
	movq	%r10, 32(%rsp)
	cmpq	%rsi, %rax
	vmovsd	%xmm1, 24(%rsp)
	cmova	%rsi, %rax
	salq	$3, %rax
	movq	%rax, %rdi
	movq	%rax, 16(%rsp)
.LEHB3:
	call	_Znwm@PLT
	vmovsd	24(%rsp), %xmm1
	movq	48(%rsp), %rdx
	movq	%rax, %rcx
	vmovsd	%xmm1, (%rax,%rdx)
.L146:
	testq	%rdx, %rdx
	movq	32(%rsp), %r10
	movq	40(%rsp), %r9
	movq	56(%rsp), %r8
	je	.L20
	movq	%rcx, %rdi
	movq	%r12, %rsi
	movq	%r8, 48(%rsp)
	movq	%rdx, 24(%rsp)
	call	memcpy@PLT
	movq	48(%rsp), %r8
	movq	40(%rsp), %r9
	movq	32(%rsp), %r10
	movq	24(%rsp), %rdx
	movq	%rax, %rcx
.L20:
	leaq	8(%rcx,%rdx), %rax
	testq	%r12, %r12
	je	.L21
	movq	%rbp, %rsi
	movq	%r12, %rdi
	movq	%rcx, 56(%rsp)
	subq	%r12, %rsi
	movq	%r8, 48(%rsp)
	movq	%r9, 40(%rsp)
	movq	%r10, 32(%rsp)
	movq	%rax, 24(%rsp)
	call	_ZdlPvm@PLT
	movq	56(%rsp), %rcx
	movq	48(%rsp), %r8
	movq	40(%rsp), %r9
	movq	32(%rsp), %r10
	movq	24(%rsp), %rax
.L21:
	movq	16(%rsp), %rbp
	movq	%rcx, %r12
	addq	%rcx, %rbp
	jmp	.L14
	.p2align 4,,10
	.p2align 3
.L18:
	movq	%rbp, %rdx
	subq	%r12, %rdx
	movq	%rdx, %rsi
	sarq	$3, %rsi
	cmpq	%r14, %rsi
	je	.L133
	testq	%rsi, %rsi
	movl	$1, %eax
	movq	%r8, 56(%rsp)
	cmovne	%rsi, %rax
	movq	%rdx, 48(%rsp)
	movq	%r9, 40(%rsp)
	addq	%rsi, %rax
	movabsq	$1152921504606846975, %rsi
	movq	%r10, 32(%rsp)
	cmpq	%rsi, %rax
	vmovsd	%xmm0, 24(%rsp)
	cmova	%rsi, %rax
	salq	$3, %rax
	movq	%rax, %rdi
	movq	%rax, 16(%rsp)
	call	_Znwm@PLT
.LEHE3:
	vmovsd	24(%rsp), %xmm0
	movq	48(%rsp), %rdx
	movq	%rax, %rcx
	vmovsd	%xmm0, (%rax,%rdx)
	jmp	.L146
	.p2align 4,,10
	.p2align 3
.L48:
	movq	%rbx, %r10
	movabsq	$1152921504606846975, %rdi
	subq	%r14, %r10
	movq	%r10, %rax
	sarq	$3, %rax
	cmpq	%rdi, %rax
	je	.L136
	testq	%rax, %rax
	movl	$1, %r13d
	movq	%rcx, 40(%rsp)
	cmovne	%rax, %r13
	movq	%r10, 32(%rsp)
	vmovsd	%xmm0, 24(%rsp)
	addq	%rax, %r13
	movabsq	$1152921504606846975, %rax
	vmovsd	%xmm1, 16(%rsp)
	cmpq	%rax, %r13
	cmova	%rax, %r13
	salq	$3, %r13
	movq	%r13, %rdi
.LEHB4:
	call	_Znwm@PLT
.LEHE4:
	movq	32(%rsp), %r10
	vmovsd	24(%rsp), %xmm0
	movq	%rax, %r9
	vmovsd	16(%rsp), %xmm1
	movq	40(%rsp), %rcx
	testq	%r10, %r10
	vmovsd	%xmm0, (%rax,%r10)
	je	.L50
	movq	%r10, %rdx
	movq	%r14, %rsi
	movq	%rax, %rdi
	movq	%rcx, 32(%rsp)
	movq	%r10, 16(%rsp)
	vmovsd	%xmm1, 24(%rsp)
	call	memcpy@PLT
	movq	32(%rsp), %rcx
	vmovsd	24(%rsp), %xmm1
	movq	16(%rsp), %r10
	movq	%rax, %r9
.L50:
	leaq	8(%r9,%r10), %rax
	testq	%r14, %r14
	je	.L51
	movq	%rbx, %rsi
	movq	%r14, %rdi
	movq	%r9, 40(%rsp)
	subq	%r14, %rsi
	movq	%rcx, 32(%rsp)
	movq	%rax, 24(%rsp)
	vmovsd	%xmm1, 16(%rsp)
	call	_ZdlPvm@PLT
	movq	40(%rsp), %r9
	movq	32(%rsp), %rcx
	movq	24(%rsp), %rax
	vmovsd	16(%rsp), %xmm1
.L51:
	leaq	(%r9,%r13), %rbx
	movq	%r9, %r14
	jmp	.L46
	.p2align 4,,10
	.p2align 3
.L150:
	cmpq	%r8, %rbx
	jb	.L35
	jmp	.L27
	.p2align 5
	.p2align 4,,10
	.p2align 3
.L156:
	vmovsd	(%r9,%rbx,8), %xmm0
	addq	$8, %rax
	vmovsd	%xmm0, -8(%rax)
.L31:
	incq	%rbx
	cmpq	%rbx, %r8
	je	.L27
.L35:
	cmpq	%rbp, %rax
	jne	.L156
	movq	%rbp, %rdx
	movabsq	$1152921504606846975, %rdi
	subq	%r12, %rdx
	movq	%rdx, %rax
	sarq	$3, %rax
	cmpq	%rdi, %rax
	je	.L134
	testq	%rax, %rax
	movl	$1, %r14d
	movq	%r8, 40(%rsp)
	cmovne	%rax, %r14
	movq	%rdx, 32(%rsp)
	movq	%r9, 24(%rsp)
	addq	%rax, %r14
	movabsq	$1152921504606846975, %rax
	movq	%r10, 16(%rsp)
	cmpq	%rax, %r14
	cmova	%rax, %r14
	salq	$3, %r14
	movq	%r14, %rdi
.LEHB5:
	call	_Znwm@PLT
.LEHE5:
	movq	24(%rsp), %r9
	movq	32(%rsp), %rdx
	movq	%rax, %r11
	movq	16(%rsp), %r10
	movq	40(%rsp), %r8
	vmovsd	(%r9,%rbx,8), %xmm0
	testq	%rdx, %rdx
	vmovsd	%xmm0, (%rax,%rdx)
	je	.L33
	movq	%r12, %rsi
	movq	%rax, %rdi
	movq	%r9, 32(%rsp)
	movq	%r10, 24(%rsp)
	movq	%rdx, 16(%rsp)
	call	memcpy@PLT
	movq	40(%rsp), %r8
	movq	32(%rsp), %r9
	movq	24(%rsp), %r10
	movq	16(%rsp), %rdx
	movq	%rax, %r11
.L33:
	leaq	8(%r11,%rdx), %rax
	testq	%r12, %r12
	je	.L34
	movq	%rbp, %rsi
	movq	%r12, %rdi
	movq	%r11, 48(%rsp)
	subq	%r12, %rsi
	movq	%r8, 40(%rsp)
	movq	%r9, 32(%rsp)
	movq	%r10, 24(%rsp)
	movq	%rax, 16(%rsp)
	call	_ZdlPvm@PLT
	movq	48(%rsp), %r11
	movq	40(%rsp), %r8
	movq	32(%rsp), %r9
	movq	24(%rsp), %r10
	movq	16(%rsp), %rax
.L34:
	leaq	(%r11,%r14), %rbp
	movq	%r11, %r12
	jmp	.L31
	.p2align 4,,10
	.p2align 3
.L148:
	movq	8(%rdx), %rax
	xorl	%ecx, %ecx
	subq	(%rdx), %rax
	movq	%rax, %rbx
	je	.L3
	movq	%rax, %rdi
	movq	%rdx, 16(%rsp)
.LEHB6:
	call	_Znwm@PLT
	movq	16(%rsp), %rdx
	movq	%rax, %rcx
.L3:
	movq	8(%rsp), %rdi
	leaq	(%rcx,%rbx), %rax
	vmovq	%rcx, %xmm6
	vpunpcklqdq	%xmm6, %xmm6, %xmm0
	movq	%rax, 16(%rdi)
	vmovdqu	%xmm0, (%rdi)
	movq	(%rdx), %rsi
	movq	8(%rdx), %rbx
	subq	%rsi, %rbx
	testq	%rbx, %rbx
	jle	.L7
	movq	%rcx, %rdi
	movq	%rbx, %rdx
	call	memcpy@PLT
	leaq	(%rax,%rbx), %rcx
.L7:
	movq	8(%rsp), %rax
	movq	%rcx, 8(%rax)
	jmp	.L1
	.p2align 4,,10
	.p2align 3
.L149:
	movq	%r8, %rdi
	movq	%rsi, 24(%rsp)
	movq	%r8, 16(%rsp)
	call	_Znwm@PLT
.LEHE6:
	movq	16(%rsp), %r8
	movq	24(%rsp), %rsi
	movq	%rax, %rcx
	vmovq	%rax, %xmm7
	movq	8(%rsp), %rax
	addq	%rcx, %r8
	vpunpcklqdq	%xmm7, %xmm7, %xmm0
	movq	%r8, 16(%rax)
	vmovdqu	%xmm0, (%rax)
	movq	(%rsi), %rax
	movq	8(%rsi), %rbx
	subq	%rax, %rbx
	testq	%rbx, %rbx
	jle	.L7
	movq	%rcx, %rdi
	movq	%rbx, %rdx
	movq	%rax, %rsi
	call	memcpy@PLT
	leaq	(%rax,%rbx), %rcx
	jmp	.L7
.L153:
	cmpq	%rax, %r14
	jne	.L56
	cmpq	%r14, %rbx
	jne	.L45
	movq	%rbx, %r13
	xorl	%ecx, %ecx
	xorl	%r15d, %r15d
	jmp	.L44
	.p2align 4,,10
	.p2align 3
.L155:
	movq	%r15, %rdx
	movq	%r13, %rsi
	movq	%rax, %rdi
	call	memcpy@PLT
	jmp	.L58
.L71:
	movq	%rax, %r14
	jmp	.L45
.L12:
	testq	%r8, %r8
	je	.L137
	movq	%rax, %r12
	xorl	%ebx, %ebx
	xorl	%r15d, %r15d
	jmp	.L35
.L154:
	movq	%rbx, %r15
	movabsq	$9223372036854775800, %rax
	subq	%r14, %r15
	movq	%r15, %rcx
	sarq	$3, %rcx
	cmpq	%rax, %r15
	je	.L57
	movq	%r14, %r13
	jmp	.L44
.L137:
	movq	%rax, %r12
	xorl	%r15d, %r15d
	testq	%r13, %r13
	jne	.L42
	jmp	.L9
.L74:
	endbr64
	movq	%rax, %r15
	jmp	.L60
.L129:
	endbr64
	jmp	.L130
.L127:
	endbr64
	jmp	.L128
	.section	.gcc_except_table,"a",@progbits
.LLSDA2287:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSE2287-.LLSDACSB2287
.LLSDACSB2287:
	.uleb128 .LEHB0-.LFB2287
	.uleb128 .LEHE0-.LEHB0
	.uleb128 0
	.uleb128 0
	.uleb128 .LEHB1-.LFB2287
	.uleb128 .LEHE1-.LEHB1
	.uleb128 .L127-.LFB2287
	.uleb128 0
	.uleb128 .LEHB2-.LFB2287
	.uleb128 .LEHE2-.LEHB2
	.uleb128 .L74-.LFB2287
	.uleb128 0
	.uleb128 .LEHB3-.LFB2287
	.uleb128 .LEHE3-.LEHB3
	.uleb128 .L127-.LFB2287
	.uleb128 0
	.uleb128 .LEHB4-.LFB2287
	.uleb128 .LEHE4-.LEHB4
	.uleb128 .L129-.LFB2287
	.uleb128 0
	.uleb128 .LEHB5-.LFB2287
	.uleb128 .LEHE5-.LEHB5
	.uleb128 .L127-.LFB2287
	.uleb128 0
	.uleb128 .LEHB6-.LFB2287
	.uleb128 .LEHE6-.LEHB6
	.uleb128 0
	.uleb128 0
.LLSDACSE2287:
	.text
	.cfi_endproc
	.section	.text.unlikely
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDAC2287
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_.cold, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_.cold:
.LFSB2287:
.L133:
	.cfi_def_cfa_offset 128
	.cfi_offset 3, -56
	.cfi_offset 6, -48
	.cfi_offset 12, -40
	.cfi_offset 13, -32
	.cfi_offset 14, -24
	.cfi_offset 15, -16
	leaq	.LC1(%rip), %rdi
.LEHB7:
	call	_ZSt20__throw_length_errorPKc@PLT
.L132:
	leaq	.LC1(%rip), %rdi
	call	_ZSt20__throw_length_errorPKc@PLT
.LEHE7:
.L72:
	endbr64
.L128:
	movq	%rax, %r15
	vzeroupper
.L63:
	testq	%r12, %r12
	je	.L65
	movq	%rbp, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L65:
	movq	%r15, %rdi
.LEHB8:
	call	_Unwind_Resume@PLT
.LEHE8:
.L134:
	leaq	.LC1(%rip), %rdi
.LEHB9:
	call	_ZSt20__throw_length_errorPKc@PLT
.LEHE9:
.L57:
	leaq	.LC1(%rip), %rdi
.LEHB10:
	call	_ZSt20__throw_length_errorPKc@PLT
.LEHE10:
.L135:
	leaq	.LC1(%rip), %rdi
.LEHB11:
	call	_ZSt20__throw_length_errorPKc@PLT
.LEHE11:
.L60:
	movq	%r13, %r14
.L61:
	testq	%r14, %r14
	je	.L140
	movq	%rbx, %rsi
	movq	%r14, %rdi
	subq	%r14, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
	jmp	.L63
.L73:
	endbr64
.L130:
	movq	%rax, %r15
	jmp	.L61
.L131:
	leaq	.LC0(%rip), %rdi
.LEHB12:
	call	_ZSt20__throw_length_errorPKc@PLT
.LEHE12:
.L136:
	leaq	.LC1(%rip), %rdi
.LEHB13:
	call	_ZSt20__throw_length_errorPKc@PLT
.LEHE13:
.L140:
	vzeroupper
	jmp	.L63
	.cfi_endproc
.LFE2287:
	.section	.gcc_except_table
.LLSDAC2287:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSEC2287-.LLSDACSBC2287
.LLSDACSBC2287:
	.uleb128 .LEHB7-.LCOLDB4
	.uleb128 .LEHE7-.LEHB7
	.uleb128 .L72-.LCOLDB4
	.uleb128 0
	.uleb128 .LEHB8-.LCOLDB4
	.uleb128 .LEHE8-.LEHB8
	.uleb128 0
	.uleb128 0
	.uleb128 .LEHB9-.LCOLDB4
	.uleb128 .LEHE9-.LEHB9
	.uleb128 .L72-.LCOLDB4
	.uleb128 0
	.uleb128 .LEHB10-.LCOLDB4
	.uleb128 .LEHE10-.LEHB10
	.uleb128 .L73-.LCOLDB4
	.uleb128 0
	.uleb128 .LEHB11-.LCOLDB4
	.uleb128 .LEHE11-.LEHB11
	.uleb128 .L72-.LCOLDB4
	.uleb128 0
	.uleb128 .LEHB12-.LCOLDB4
	.uleb128 .LEHE12-.LEHB12
	.uleb128 0
	.uleb128 0
	.uleb128 .LEHB13-.LCOLDB4
	.uleb128 .LEHE13-.LEHB13
	.uleb128 .L73-.LCOLDB4
	.uleb128 0
.LLSDACSEC2287:
	.section	.text.unlikely
	.text
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
	.section	.text.unlikely
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_.cold, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_.cold
.LCOLDE4:
	.text
.LHOTE4:
	.section	.text.unlikely
.LCOLDB6:
	.text
.LHOTB6:
	.p2align 4
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd:
.LFB2245:
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDA2245
	vsubsd	%xmm1, %xmm0, %xmm3
	pushq	%r14
	.cfi_def_cfa_offset 16
	.cfi_offset 14, -16
	vsubsd	%xmm3, %xmm0, %xmm2
	pushq	%rbx
	.cfi_def_cfa_offset 24
	.cfi_offset 3, -24
	movq	%rdi, %rbx
	vaddsd	%xmm2, %xmm3, %xmm4
	vsubsd	%xmm1, %xmm2, %xmm2
	vpxor	%xmm1, %xmm1, %xmm1
	subq	$24, %rsp
	.cfi_def_cfa_offset 48
	vmovdqu	%xmm1, (%rdi)
	vxorpd	%xmm1, %xmm1, %xmm1
	movq	$0, 16(%rdi)
	vsubsd	%xmm4, %xmm0, %xmm0
	vaddsd	%xmm2, %xmm0, %xmm0
	vucomisd	%xmm1, %xmm0
	jp	.L170
	jne	.L170
	vucomisd	%xmm1, %xmm3
	jp	.L171
	je	.L157
.L171:
	movl	$8, %edi
	vmovsd	%xmm3, (%rsp)
.LEHB14:
	call	_Znwm@PLT
	vmovsd	(%rsp), %xmm3
	xorl	%esi, %esi
	movl	$8, %ecx
	xorl	%edx, %edx
	vmovsd	%xmm3, (%rax)
.L165:
	addq	%rax, %rcx
	testq	%rdx, %rdx
	je	.L166
	movq	%rdx, %rdi
	movq	%rcx, 8(%rsp)
	movq	%rax, (%rsp)
	call	_ZdlPvm@PLT
	movq	8(%rsp), %rcx
	movq	(%rsp), %rax
.L166:
	movq	%rax, (%rbx)
	movq	%rcx, 8(%rbx)
	movq	%rcx, 16(%rbx)
.L157:
	addq	$24, %rsp
	.cfi_remember_state
	.cfi_def_cfa_offset 24
	movq	%rbx, %rax
	popq	%rbx
	.cfi_def_cfa_offset 16
	popq	%r14
	.cfi_def_cfa_offset 8
	ret
	.p2align 4,,10
	.p2align 3
.L170:
	.cfi_restore_state
	movl	$8, %edi
	vmovsd	%xmm0, 8(%rsp)
	vmovsd	%xmm3, (%rsp)
	call	_Znwm@PLT
	vmovsd	8(%rsp), %xmm0
	vmovsd	(%rsp), %xmm3
	movq	%rax, %rdx
	leaq	8(%rax), %rax
	movq	%rdx, (%rbx)
	vmovsd	%xmm0, -8(%rax)
	vxorpd	%xmm0, %xmm0, %xmm0
	vucomisd	%xmm0, %xmm3
	movq	%rax, 8(%rbx)
	movq	%rax, 16(%rbx)
	jp	.L160
	je	.L157
.L160:
	movl	$16, %edi
	movq	%rdx, 8(%rsp)
	vmovsd	%xmm3, (%rsp)
	call	_Znwm@PLT
.LEHE14:
	movq	8(%rsp), %rdx
	vmovsd	(%rsp), %xmm3
	movl	$8, %esi
	movq	(%rdx), %rcx
	vmovsd	%xmm3, 8(%rax)
	movq	%rcx, (%rax)
	movl	$16, %ecx
	jmp	.L165
.L169:
	endbr64
	movq	%rax, %r14
	jmp	.L167
	.section	.gcc_except_table
.LLSDA2245:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSE2245-.LLSDACSB2245
.LLSDACSB2245:
	.uleb128 .LEHB14-.LFB2245
	.uleb128 .LEHE14-.LEHB14
	.uleb128 .L169-.LFB2245
	.uleb128 0
.LLSDACSE2245:
	.text
	.cfi_endproc
	.section	.text.unlikely
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDAC2245
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd.cold, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd.cold:
.LFSB2245:
.L167:
	.cfi_def_cfa_offset 48
	.cfi_offset 3, -24
	.cfi_offset 14, -16
	movq	(%rbx), %rdi
	testq	%rdi, %rdi
	je	.L178
	movq	16(%rbx), %rsi
	subq	%rdi, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
.L168:
	movq	%r14, %rdi
.LEHB15:
	call	_Unwind_Resume@PLT
.LEHE15:
.L178:
	vzeroupper
	jmp	.L168
	.cfi_endproc
.LFE2245:
	.section	.gcc_except_table
.LLSDAC2245:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSEC2245-.LLSDACSBC2245
.LLSDACSBC2245:
	.uleb128 .LEHB15-.LCOLDB6
	.uleb128 .LEHE15-.LEHB15
	.uleb128 0
	.uleb128 0
.LLSDACSEC2245:
	.section	.text.unlikely
	.text
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd
	.section	.text.unlikely
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd.cold, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd.cold
.LCOLDE6:
	.text
.LHOTE6:
	.section	.text.unlikely
.LCOLDB8:
	.text
.LHOTB8:
	.p2align 4
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd:
.LFB2310:
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDA2310
	pushq	%r14
	.cfi_def_cfa_offset 16
	.cfi_offset 14, -16
	vmulsd	%xmm1, %xmm0, %xmm4
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm1, %xmm5
	pushq	%rbx
	.cfi_def_cfa_offset 24
	.cfi_offset 3, -24
	movq	%rdi, %rbx
	subq	$24, %rsp
	.cfi_def_cfa_offset 48
	movq	$0, 16(%rdi)
	vmovsd	.LC7(%rip), %xmm2
	vfmsub132sd	%xmm2, %xmm0, %xmm3
	vfmsub132sd	%xmm2, %xmm1, %xmm5
	vfmsub231sd	%xmm2, %xmm0, %xmm3
	vfmsub132sd	%xmm1, %xmm5, %xmm2
	vmovapd	%xmm3, %xmm5
	vsubsd	%xmm3, %xmm0, %xmm0
	vsubsd	%xmm2, %xmm1, %xmm1
	vfnmadd132sd	%xmm2, %xmm4, %xmm5
	vfnmadd132sd	%xmm0, %xmm5, %xmm2
	vfnmadd132sd	%xmm1, %xmm2, %xmm3
	vfmsub132sd	%xmm1, %xmm3, %xmm0
	vpxor	%xmm1, %xmm1, %xmm1
	vmovdqu	%xmm1, (%rdi)
	vxorpd	%xmm1, %xmm1, %xmm1
	vucomisd	%xmm1, %xmm0
	jp	.L193
	jne	.L193
	vucomisd	%xmm1, %xmm4
	jp	.L194
	je	.L180
.L194:
	movl	$8, %edi
	vmovsd	%xmm4, (%rsp)
.LEHB16:
	call	_Znwm@PLT
	vmovsd	(%rsp), %xmm4
	xorl	%esi, %esi
	movl	$8, %ecx
	xorl	%edx, %edx
	vmovsd	%xmm4, (%rax)
.L188:
	addq	%rax, %rcx
	testq	%rdx, %rdx
	je	.L189
	movq	%rdx, %rdi
	movq	%rcx, 8(%rsp)
	movq	%rax, (%rsp)
	call	_ZdlPvm@PLT
	movq	8(%rsp), %rcx
	movq	(%rsp), %rax
.L189:
	movq	%rax, (%rbx)
	movq	%rcx, 8(%rbx)
	movq	%rcx, 16(%rbx)
.L180:
	addq	$24, %rsp
	.cfi_remember_state
	.cfi_def_cfa_offset 24
	movq	%rbx, %rax
	popq	%rbx
	.cfi_def_cfa_offset 16
	popq	%r14
	.cfi_def_cfa_offset 8
	ret
	.p2align 4,,10
	.p2align 3
.L193:
	.cfi_restore_state
	movl	$8, %edi
	vmovsd	%xmm0, 8(%rsp)
	vmovsd	%xmm4, (%rsp)
	call	_Znwm@PLT
	vmovsd	8(%rsp), %xmm0
	vmovsd	(%rsp), %xmm4
	movq	%rax, %rdx
	leaq	8(%rax), %rax
	movq	%rdx, (%rbx)
	vmovsd	%xmm0, -8(%rax)
	vxorpd	%xmm0, %xmm0, %xmm0
	vucomisd	%xmm0, %xmm4
	movq	%rax, 8(%rbx)
	movq	%rax, 16(%rbx)
	jp	.L183
	je	.L180
.L183:
	movl	$16, %edi
	movq	%rdx, 8(%rsp)
	vmovsd	%xmm4, (%rsp)
	call	_Znwm@PLT
.LEHE16:
	movq	8(%rsp), %rdx
	vmovsd	(%rsp), %xmm4
	movl	$8, %esi
	movq	(%rdx), %rcx
	vmovsd	%xmm4, 8(%rax)
	movq	%rcx, (%rax)
	movl	$16, %ecx
	jmp	.L188
.L192:
	endbr64
	movq	%rax, %r14
	jmp	.L190
	.section	.gcc_except_table
.LLSDA2310:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSE2310-.LLSDACSB2310
.LLSDACSB2310:
	.uleb128 .LEHB16-.LFB2310
	.uleb128 .LEHE16-.LEHB16
	.uleb128 .L192-.LFB2310
	.uleb128 0
.LLSDACSE2310:
	.text
	.cfi_endproc
	.section	.text.unlikely
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDAC2310
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd.cold, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd.cold:
.LFSB2310:
.L190:
	.cfi_def_cfa_offset 48
	.cfi_offset 3, -24
	.cfi_offset 14, -16
	movq	(%rbx), %rdi
	testq	%rdi, %rdi
	je	.L201
	movq	16(%rbx), %rsi
	subq	%rdi, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
.L191:
	movq	%r14, %rdi
.LEHB17:
	call	_Unwind_Resume@PLT
.LEHE17:
.L201:
	vzeroupper
	jmp	.L191
	.cfi_endproc
.LFE2310:
	.section	.gcc_except_table
.LLSDAC2310:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSEC2310-.LLSDACSBC2310
.LLSDACSBC2310:
	.uleb128 .LEHB17-.LCOLDB8
	.uleb128 .LEHE17-.LEHB17
	.uleb128 0
	.uleb128 0
.LLSDACSEC2310:
	.section	.text.unlikely
	.text
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd
	.section	.text.unlikely
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd.cold, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd.cold
.LCOLDE8:
	.text
.LHOTE8:
	.section	.text.unlikely
.LCOLDB16:
	.text
.LHOTB16:
	.p2align 4
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13orient2dExactENS0_4Vec2ES3_S3_, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13orient2dExactENS0_4Vec2ES3_S3_:
.LFB2311:
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDA2311
	leaq	8(%rsp), %r10
	.cfi_def_cfa 10, 0
	andq	$-32, %rsp
	pushq	-8(%r10)
	pushq	%rbp
	movq	%rsp, %rbp
	.cfi_escape 0x10,0x6,0x2,0x76,0
	pushq	%r15
	.cfi_escape 0x10,0xf,0x2,0x76,0x78
	leaq	-144(%rbp), %r15
	pushq	%r14
	pushq	%r13
	pushq	%r12
	pushq	%r10
	.cfi_escape 0xf,0x3,0x76,0x58,0x6
	.cfi_escape 0x10,0xe,0x2,0x76,0x70
	.cfi_escape 0x10,0xd,0x2,0x76,0x68
	.cfi_escape 0x10,0xc,0x2,0x76,0x60
	pushq	%rbx
	subq	$224, %rsp
	.cfi_escape 0x10,0x3,0x2,0x76,0x50
	vmovsd	%xmm1, -208(%rbp)
	vmovapd	%xmm3, %xmm1
	vmovsd	%xmm2, -216(%rbp)
	vmovsd	%xmm4, -224(%rbp)
	vmovsd	%xmm5, -232(%rbp)
	movq	%fs:40, %rdi
	movq	%rdi, -56(%rbp)
	leaq	-176(%rbp), %rdi
	vmovsd	%xmm0, -184(%rbp)
	vmovsd	%xmm3, -200(%rbp)
	movq	%rdi, -192(%rbp)
.LEHB18:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd
.LEHE18:
	vmovsd	-232(%rbp), %xmm1
	vmovsd	-184(%rbp), %xmm0
	movq	%r15, %rdi
	movq	-176(%rbp), %rbx
	movq	-160(%rbp), %r14
.LEHB19:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd
.LEHE19:
	vmovdqa	-144(%rbp), %xmm2
	vmovsd	-200(%rbp), %xmm3
	vpextrq	$1, %xmm2, %rax
	vmovq	%xmm2, %r12
	cmpq	%rax, %r12
	je	.L204
	subq	$8, %rax
	subq	%r12, %rax
	movq	%rax, %rsi
	shrq	$3, %rsi
	leaq	1(%rsi), %rcx
	cmpq	$16, %rax
	jbe	.L261
	movq	%rcx, %rdx
	vbroadcastsd	.LC10(%rip), %ymm0
	movq	%r12, %rax
	shrq	$2, %rdx
	salq	$5, %rdx
	addq	%r12, %rdx
	.p2align 5
	.p2align 4,,10
	.p2align 3
.L206:
	vxorpd	(%rax), %ymm0, %ymm1
	addq	$32, %rax
	vmovupd	%ymm1, -32(%rax)
	cmpq	%rdx, %rax
	jne	.L206
	testb	$3, %cl
	je	.L374
	movq	%rcx, %rax
	andq	$-4, %rax
	leaq	(%r12,%rax,8), %rdx
	vzeroupper
.L205:
	cmpq	%rax, %rsi
	je	.L208
	vmovddup	.LC10(%rip), %xmm0
	movq	%rcx, %rsi
	subq	%rax, %rsi
	andl	$1, %ecx
	vxorpd	(%r12,%rax,8), %xmm0, %xmm0
	vmovupd	%xmm0, (%r12,%rax,8)
	je	.L204
	andq	$-2, %rsi
	leaq	(%rdx,%rsi,8), %rdx
.L208:
	vmovsd	(%rdx), %xmm0
	vxorpd	.LC12(%rip), %xmm0, %xmm0
	vmovsd	%xmm0, (%rdx)
.L204:
	movq	-128(%rbp), %rax
	movq	-192(%rbp), %rsi
	leaq	-80(%rbp), %rdi
	movq	%rbx, -176(%rbp)
	movq	%r14, -160(%rbp)
	movq	%rax, -256(%rbp)
	movq	%rax, -96(%rbp)
	leaq	-112(%rbp), %rax
	movq	%rax, %rdx
	movq	%rdi, -200(%rbp)
	movq	%rax, -184(%rbp)
	vmovsd	%xmm3, -248(%rbp)
	vmovdqa	%xmm2, -112(%rbp)
.LEHB20:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE20:
	movq	-72(%rbp), %rax
	testq	%rbx, %rbx
	movq	-80(%rbp), %r13
	vmovsd	-248(%rbp), %xmm3
	movq	%rax, -168(%rbp)
	movq	-64(%rbp), %rax
	movq	%rax, -240(%rbp)
	je	.L210
	movq	%r14, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
	vmovsd	-248(%rbp), %xmm3
.L210:
	testq	%r12, %r12
	je	.L211
	movq	-256(%rbp), %rsi
	movq	%r12, %rdi
	vmovsd	%xmm3, -248(%rbp)
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
	vmovsd	-248(%rbp), %xmm3
.L211:
	vmovsd	-224(%rbp), %xmm0
	vmovapd	%xmm3, %xmm1
	movq	%r15, %rdi
.LEHB21:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd
.LEHE21:
	vmovdqa	-144(%rbp), %xmm2
	vpextrq	$1, %xmm2, %rax
	vmovq	%xmm2, %r12
	cmpq	%rax, %r12
	je	.L215
	subq	$8, %rax
	subq	%r12, %rax
	movq	%rax, %rsi
	shrq	$3, %rsi
	leaq	1(%rsi), %rcx
	cmpq	$16, %rax
	jbe	.L262
	movq	%rcx, %rdx
	vbroadcastsd	.LC10(%rip), %ymm0
	movq	%r12, %rax
	shrq	$2, %rdx
	salq	$5, %rdx
	addq	%r12, %rdx
	.p2align 5
	.p2align 4,,10
	.p2align 3
.L217:
	vxorpd	(%rax), %ymm0, %ymm1
	addq	$32, %rax
	vmovupd	%ymm1, -32(%rax)
	cmpq	%rdx, %rax
	jne	.L217
	testb	$3, %cl
	je	.L375
	movq	%rcx, %rax
	andq	$-4, %rax
	leaq	(%r12,%rax,8), %rdx
	vzeroupper
.L216:
	cmpq	%rax, %rsi
	je	.L219
	vmovddup	.LC10(%rip), %xmm0
	movq	%rcx, %rsi
	subq	%rax, %rsi
	andl	$1, %ecx
	vxorpd	(%r12,%rax,8), %xmm0, %xmm0
	vmovupd	%xmm0, (%r12,%rax,8)
	je	.L215
	andq	$-2, %rsi
	leaq	(%rdx,%rsi,8), %rdx
.L219:
	vmovsd	(%rdx), %xmm0
	vxorpd	.LC12(%rip), %xmm0, %xmm0
	vmovsd	%xmm0, (%rdx)
.L215:
	movq	-128(%rbp), %rax
	movq	-184(%rbp), %rdx
	movq	%r13, -176(%rbp)
	movq	-192(%rbp), %rsi
	movq	-200(%rbp), %rdi
	vmovdqa	%xmm2, -112(%rbp)
	movq	%rax, -248(%rbp)
	movq	%rax, -96(%rbp)
	movq	-240(%rbp), %rax
	movq	%rax, -160(%rbp)
.LEHB22:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE22:
	movq	-72(%rbp), %rax
	movq	-80(%rbp), %rbx
	movq	-64(%rbp), %r14
	movq	%rax, -168(%rbp)
	testq	%r13, %r13
	je	.L221
	movq	-240(%rbp), %rsi
	movq	%r13, %rdi
	subq	%r13, %rsi
	call	_ZdlPvm@PLT
.L221:
	testq	%r12, %r12
	je	.L222
	movq	-248(%rbp), %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L222:
	vmovsd	-216(%rbp), %xmm1
	vmovsd	-208(%rbp), %xmm0
	movq	%r15, %rdi
.LEHB23:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd
.LEHE23:
	vmovdqa	-144(%rbp), %xmm2
	vpextrq	$1, %xmm2, %rax
	vmovq	%xmm2, %r12
	cmpq	%rax, %r12
	je	.L223
	subq	$8, %rax
	subq	%r12, %rax
	movq	%rax, %rsi
	shrq	$3, %rsi
	leaq	1(%rsi), %rcx
	cmpq	$16, %rax
	jbe	.L263
	movq	%rcx, %rdx
	vbroadcastsd	.LC10(%rip), %ymm0
	movq	%r12, %rax
	shrq	$2, %rdx
	salq	$5, %rdx
	addq	%r12, %rdx
	.p2align 5
	.p2align 4,,10
	.p2align 3
.L225:
	vxorpd	(%rax), %ymm0, %ymm1
	addq	$32, %rax
	vmovupd	%ymm1, -32(%rax)
	cmpq	%rdx, %rax
	jne	.L225
	testb	$3, %cl
	je	.L376
	movq	%rcx, %rax
	andq	$-4, %rax
	leaq	(%r12,%rax,8), %rdx
	vzeroupper
.L224:
	cmpq	%rax, %rsi
	je	.L227
	vmovddup	.LC10(%rip), %xmm0
	movq	%rcx, %rsi
	subq	%rax, %rsi
	andl	$1, %ecx
	vxorpd	(%r12,%rax,8), %xmm0, %xmm0
	vmovupd	%xmm0, (%r12,%rax,8)
	je	.L223
	andq	$-2, %rsi
	leaq	(%rdx,%rsi,8), %rdx
.L227:
	vmovsd	(%rdx), %xmm0
	vxorpd	.LC12(%rip), %xmm0, %xmm0
	vmovsd	%xmm0, (%rdx)
.L223:
	movq	-128(%rbp), %rax
	movq	-184(%rbp), %rdx
	movq	%rbx, -176(%rbp)
	movq	-192(%rbp), %rsi
	movq	-200(%rbp), %rdi
	movq	%r14, -160(%rbp)
	movq	%rax, -240(%rbp)
	movq	%rax, -96(%rbp)
	vmovdqa	%xmm2, -112(%rbp)
.LEHB24:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE24:
	movq	-72(%rbp), %rax
	movq	-80(%rbp), %r13
	movq	-64(%rbp), %r15
	movq	%rax, -168(%rbp)
	testq	%rbx, %rbx
	je	.L229
	movq	%r14, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L229:
	testq	%r12, %r12
	je	.L230
	movq	-240(%rbp), %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L230:
	vmovsd	-224(%rbp), %xmm1
	vmovsd	-208(%rbp), %xmm0
	movq	-184(%rbp), %rdi
.LEHB25:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd
.LEHE25:
	movq	-96(%rbp), %rax
	movq	-112(%rbp), %r12
	movq	%r13, -176(%rbp)
	movq	-184(%rbp), %rdx
	movq	-192(%rbp), %rsi
	movq	%r15, -160(%rbp)
	movq	-200(%rbp), %rdi
	movq	%rax, -208(%rbp)
	movq	%r12, -112(%rbp)
	movq	%rax, -96(%rbp)
.LEHB26:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE26:
	movq	-72(%rbp), %rax
	movq	-80(%rbp), %rbx
	movq	-64(%rbp), %r14
	movq	%rax, -168(%rbp)
	testq	%r13, %r13
	je	.L233
	movq	%r15, %rsi
	movq	%r13, %rdi
	subq	%r13, %rsi
	call	_ZdlPvm@PLT
.L233:
	testq	%r12, %r12
	je	.L234
	movq	-208(%rbp), %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L234:
	movq	-184(%rbp), %r12
	vmovsd	-216(%rbp), %xmm1
	vmovsd	-232(%rbp), %xmm0
	movq	%r12, %rdi
.LEHB27:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11productTermEdd
.LEHE27:
	movq	-112(%rbp), %r13
	movq	-96(%rbp), %r15
	movq	%r12, %rdx
	movq	%rbx, -176(%rbp)
	movq	-192(%rbp), %rsi
	movq	-200(%rbp), %rdi
	movq	%r14, -160(%rbp)
	movq	%r13, -112(%rbp)
	movq	%r15, -96(%rbp)
.LEHB28:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE28:
	movq	-64(%rbp), %rax
	movq	-80(%rbp), %r8
	movq	-72(%rbp), %r12
	movq	%rax, -184(%rbp)
	testq	%rbx, %rbx
	je	.L235
	movq	%r14, %rsi
	movq	%rbx, %rdi
	movq	%r8, -192(%rbp)
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
	movq	-192(%rbp), %r8
.L235:
	testq	%r13, %r13
	je	.L236
	movq	%r15, %rsi
	movq	%r13, %rdi
	movq	%r8, -192(%rbp)
	subq	%r13, %rsi
	call	_ZdlPvm@PLT
	movq	-192(%rbp), %r8
.L236:
	subq	%r8, %r12
	movq	%r12, %rcx
	sarq	$3, %rcx
	movq	%rcx, %rdx
	cmpq	$40, %r12
	jbe	.L237
	leaq	-32(%r8,%r12), %rax
	testb	$31, %al
	jne	.L237
	vmovq	%rcx, %xmm6
	incq	%rcx
	vpbroadcastq	.LC15(%rip), %ymm4
	xorl	%edx, %edx
	vpbroadcastq	%xmm6, %ymm1
	shrq	$2, %rcx
	vxorpd	%xmm3, %xmm3, %xmm3
	vpaddq	.LC14(%rip), %ymm1, %ymm1
	decq	%rcx
	jmp	.L240
	.p2align 4,,10
	.p2align 3
.L245:
	vpermpd	$27, (%rax), %ymm0
	vcmpltpd	%ymm0, %ymm3, %ymm2
	vptest	%ymm2, %ymm2
	jne	.L373
	vcmpltpd	%ymm3, %ymm0, %ymm0
	subq	$32, %rax
	incq	%rdx
	vptest	%ymm0, %ymm0
	jne	.L373
	vpaddq	%ymm4, %ymm1, %ymm1
.L240:
	cmpq	%rdx, %rcx
	jne	.L245
.L373:
	vmovq	%xmm1, %rax
	testq	%rax, %rax
	je	.L286
	leaq	-1(%rax), %rdx
	vxorpd	%xmm0, %xmm0, %xmm0
	vmovsd	(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L284
	vcomisd	%xmm1, %xmm0
	ja	.L285
	testq	%rdx, %rdx
	je	.L286
	vmovsd	-8(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L284
	vcomisd	%xmm1, %xmm0
	ja	.L285
	cmpq	$2, %rax
	je	.L286
	vmovsd	-16(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L284
	vcomisd	%xmm1, %xmm0
	ja	.L285
	cmpq	$3, %rax
	je	.L286
	vmovsd	-24(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L284
	vcomisd	%xmm1, %xmm0
	ja	.L285
	cmpq	$4, %rax
	je	.L286
	vmovsd	-32(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L284
	vcomisd	%xmm1, %xmm0
	ja	.L285
	cmpq	$5, %rax
	je	.L286
	vmovsd	-40(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L284
	vcomisd	%xmm1, %xmm0
	ja	.L285
	cmpq	$6, %rax
	je	.L286
	vmovsd	-48(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L284
	vcomisd	%xmm1, %xmm0
	ja	.L285
	cmpq	$7, %rax
	je	.L286
	vmovsd	-56(%r8,%rdx,8), %xmm1
	xorl	%ebx, %ebx
	vcomisd	%xmm0, %xmm1
	seta	%bl
	leal	-1(%rbx,%rbx), %ebx
	vzeroupper
.L242:
	testq	%r8, %r8
	jne	.L243
.L203:
	movq	-56(%rbp), %rax
	subq	%fs:40, %rax
	jne	.L383
	addq	$224, %rsp
	movl	%ebx, %eax
	popq	%rbx
	popq	%r10
	.cfi_remember_state
	.cfi_def_cfa 10, 0
	popq	%r12
	popq	%r13
	popq	%r14
	popq	%r15
	popq	%rbp
	leaq	-8(%r10), %rsp
	.cfi_def_cfa 7, 8
	ret
.L289:
	.cfi_restore_state
	movl	$1, %ebx
	.p2align 4,,10
	.p2align 3
.L243:
	movq	-184(%rbp), %rsi
	movq	%r8, %rdi
	subq	%r8, %rsi
	call	_ZdlPvm@PLT
	jmp	.L203
	.p2align 4,,10
	.p2align 3
.L375:
	vzeroupper
	jmp	.L215
	.p2align 4,,10
	.p2align 3
.L376:
	vzeroupper
	jmp	.L223
	.p2align 4,,10
	.p2align 3
.L374:
	vzeroupper
	jmp	.L204
	.p2align 4,,10
	.p2align 3
.L286:
	xorl	%ebx, %ebx
	vzeroupper
	jmp	.L242
	.p2align 4,,10
	.p2align 3
.L237:
	vxorpd	%xmm1, %xmm1, %xmm1
	jmp	.L244
	.p2align 4,,10
	.p2align 3
.L385:
	decq	%rdx
	vmovsd	(%r8,%rdx,8), %xmm0
	vcomisd	%xmm1, %xmm0
	ja	.L289
	vcomisd	%xmm0, %xmm1
	ja	.L384
.L244:
	testq	%rdx, %rdx
	jne	.L385
	xorl	%ebx, %ebx
	jmp	.L242
	.p2align 4,,10
	.p2align 3
.L284:
	movl	$1, %ebx
	vzeroupper
	jmp	.L242
	.p2align 4,,10
	.p2align 3
.L285:
	movl	$-1, %ebx
	vzeroupper
	jmp	.L242
.L263:
	movq	%r12, %rdx
	xorl	%eax, %eax
	jmp	.L224
.L261:
	movq	%r12, %rdx
	xorl	%eax, %eax
	jmp	.L205
.L262:
	movq	%r12, %rdx
	xorl	%eax, %eax
	jmp	.L216
.L384:
	movl	$-1, %ebx
	jmp	.L243
.L383:
	call	__stack_chk_fail@PLT
.L291:
	endbr64
	movq	%rax, %r13
	jmp	.L248
.L292:
	endbr64
	movq	%rax, %rbx
	jmp	.L250
.L297:
	endbr64
	movq	%rax, %r12
	jmp	.L232
.L293:
	endbr64
	movq	%rax, %r13
	jmp	.L252
.L295:
	endbr64
	movq	%rax, %r12
	jmp	.L256
.L294:
	endbr64
	movq	%rax, %rbx
	jmp	.L254
.L298:
	endbr64
	movq	%rax, %r12
	jmp	.L213
.L296:
	endbr64
	movq	%rax, %r12
	vzeroupper
	jmp	.L214
	.section	.gcc_except_table
.LLSDA2311:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSE2311-.LLSDACSB2311
.LLSDACSB2311:
	.uleb128 .LEHB18-.LFB2311
	.uleb128 .LEHE18-.LEHB18
	.uleb128 0
	.uleb128 0
	.uleb128 .LEHB19-.LFB2311
	.uleb128 .LEHE19-.LEHB19
	.uleb128 .L296-.LFB2311
	.uleb128 0
	.uleb128 .LEHB20-.LFB2311
	.uleb128 .LEHE20-.LEHB20
	.uleb128 .L291-.LFB2311
	.uleb128 0
	.uleb128 .LEHB21-.LFB2311
	.uleb128 .LEHE21-.LEHB21
	.uleb128 .L298-.LFB2311
	.uleb128 0
	.uleb128 .LEHB22-.LFB2311
	.uleb128 .LEHE22-.LEHB22
	.uleb128 .L292-.LFB2311
	.uleb128 0
	.uleb128 .LEHB23-.LFB2311
	.uleb128 .LEHE23-.LEHB23
	.uleb128 .L296-.LFB2311
	.uleb128 0
	.uleb128 .LEHB24-.LFB2311
	.uleb128 .LEHE24-.LEHB24
	.uleb128 .L293-.LFB2311
	.uleb128 0
	.uleb128 .LEHB25-.LFB2311
	.uleb128 .LEHE25-.LEHB25
	.uleb128 .L297-.LFB2311
	.uleb128 0
	.uleb128 .LEHB26-.LFB2311
	.uleb128 .LEHE26-.LEHB26
	.uleb128 .L294-.LFB2311
	.uleb128 0
	.uleb128 .LEHB27-.LFB2311
	.uleb128 .LEHE27-.LEHB27
	.uleb128 .L296-.LFB2311
	.uleb128 0
	.uleb128 .LEHB28-.LFB2311
	.uleb128 .LEHE28-.LEHB28
	.uleb128 .L295-.LFB2311
	.uleb128 0
.LLSDACSE2311:
	.text
	.cfi_endproc
	.section	.text.unlikely
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDAC2311
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13orient2dExactENS0_4Vec2ES3_S3_.cold, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13orient2dExactENS0_4Vec2ES3_S3_.cold:
.LFSB2311:
.L248:
	.cfi_escape 0xf,0x3,0x76,0x58,0x6
	.cfi_escape 0x10,0x3,0x2,0x76,0x50
	.cfi_escape 0x10,0x6,0x2,0x76,0
	.cfi_escape 0x10,0xc,0x2,0x76,0x60
	.cfi_escape 0x10,0xd,0x2,0x76,0x68
	.cfi_escape 0x10,0xe,0x2,0x76,0x70
	.cfi_escape 0x10,0xf,0x2,0x76,0x78
	testq	%r12, %r12
	jne	.L386
	vzeroupper
.L249:
	movq	%r13, %r12
.L214:
	testq	%rbx, %rbx
	je	.L258
	movq	%r14, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L258:
	movq	-56(%rbp), %rax
	subq	%fs:40, %rax
	jne	.L387
	movq	%r12, %rdi
.LEHB29:
	call	_Unwind_Resume@PLT
.LEHE29:
.L250:
	testq	%r12, %r12
	jne	.L388
	vzeroupper
.L251:
	movq	%rbx, %r12
	movq	-240(%rbp), %r14
	movq	%r13, %rbx
	jmp	.L214
.L386:
	movq	-256(%rbp), %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
	jmp	.L249
.L232:
	movq	%r15, %r14
	movq	%r13, %rbx
	vzeroupper
	jmp	.L214
.L252:
	testq	%r12, %r12
	jne	.L389
	vzeroupper
.L253:
	movq	%r13, %r12
	jmp	.L214
.L256:
	testq	%r13, %r13
	je	.L381
	movq	%r15, %rsi
	movq	%r13, %rdi
	subq	%r13, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
	jmp	.L214
.L254:
	testq	%r12, %r12
	jne	.L390
	vzeroupper
.L255:
	movq	%rbx, %r12
	movq	%r15, %r14
	movq	%r13, %rbx
	jmp	.L214
.L381:
	vzeroupper
	jmp	.L214
.L390:
	movq	-208(%rbp), %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
	jmp	.L255
.L213:
	movq	-240(%rbp), %r14
	movq	%r13, %rbx
	vzeroupper
	jmp	.L214
.L387:
	call	__stack_chk_fail@PLT
.L388:
	movq	-248(%rbp), %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
	jmp	.L251
.L389:
	movq	-240(%rbp), %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
	jmp	.L253
	.cfi_endproc
.LFE2311:
	.section	.gcc_except_table
.LLSDAC2311:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSEC2311-.LLSDACSBC2311
.LLSDACSBC2311:
	.uleb128 .LEHB29-.LCOLDB16
	.uleb128 .LEHE29-.LEHB29
	.uleb128 0
	.uleb128 0
.LLSDACSEC2311:
	.section	.text.unlikely
	.text
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13orient2dExactENS0_4Vec2ES3_S3_, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13orient2dExactENS0_4Vec2ES3_S3_
	.section	.text.unlikely
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13orient2dExactENS0_4Vec2ES3_S3_.cold, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13orient2dExactENS0_4Vec2ES3_S3_.cold
.LCOLDE16:
	.text
.LHOTE16:
	.section	.text.unlikely
.LCOLDB17:
	.text
.LHOTB17:
	.p2align 4
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_:
.LFB2296:
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDA2296
	pushq	%r15
	.cfi_def_cfa_offset 16
	.cfi_offset 15, -16
	vpxor	%xmm0, %xmm0, %xmm0
	pushq	%r14
	.cfi_def_cfa_offset 24
	.cfi_offset 14, -24
	pushq	%r13
	.cfi_def_cfa_offset 32
	.cfi_offset 13, -32
	pushq	%r12
	.cfi_def_cfa_offset 40
	.cfi_offset 12, -40
	movq	%rdi, %r12
	pushq	%rbp
	.cfi_def_cfa_offset 48
	.cfi_offset 6, -48
	pushq	%rbx
	.cfi_def_cfa_offset 56
	.cfi_offset 3, -56
	subq	$168, %rsp
	.cfi_def_cfa_offset 224
	movq	%rsi, 8(%rsp)
	movq	%fs:40, %rax
	movq	%rax, 152(%rsp)
	xorl	%eax, %eax
	movq	$0, 16(%rdi)
	vmovdqu	%xmm0, (%rdi)
	movq	8(%rdx), %rax
	movq	(%rdx), %r13
	movq	%rax, 16(%rsp)
	cmpq	%r13, %rax
	je	.L391
	.p2align 4,,10
	.p2align 3
.L435:
	movq	8(%rsp), %rax
	vmovsd	0(%r13), %xmm6
	movq	8(%rax), %r14
	movq	(%rax), %r8
	cmpq	%r8, %r14
	sete	%dl
	je	.L441
	vxorpd	%xmm5, %xmm5, %xmm5
	vucomisd	%xmm5, %xmm6
	setnp	%al
	cmovne	%edx, %eax
	testb	%al, %al
	jne	.L441
	subq	%r8, %r14
	movabsq	$1152921504606846975, %rsi
	movq	%r14, %r15
	sarq	$3, %r15
	leaq	(%r15,%r15), %rax
	cmpq	%rax, %rsi
	jb	.L492
	testq	%rax, %rax
	je	.L442
	leaq	(%r14,%r14), %rbp
	movq	%r8, 32(%rsp)
	movq	%rbp, %rdi
	vmovsd	%xmm6, 24(%rsp)
.LEHB30:
	call	_Znwm@PLT
.LEHE30:
	vmovsd	24(%rsp), %xmm6
	movq	32(%rsp), %r8
	movq	%rax, %rbx
	addq	%rax, %rbp
.L396:
	vmovsd	(%r8), %xmm0
	vmovsd	.LC7(%rip), %xmm1
	vmovsd	.LC7(%rip), %xmm4
	vmulsd	%xmm0, %xmm6, %xmm3
	vfmsub132sd	%xmm0, %xmm0, %xmm1
	vfmsub132sd	%xmm6, %xmm6, %xmm4
	vfmsub231sd	.LC7(%rip), %xmm0, %xmm1
	vfmsub231sd	.LC7(%rip), %xmm6, %xmm4
	vmovapd	%xmm1, %xmm2
	vsubsd	%xmm1, %xmm0, %xmm0
	vfnmadd132sd	%xmm4, %xmm3, %xmm2
	vsubsd	%xmm4, %xmm6, %xmm5
	vfnmadd231sd	%xmm4, %xmm0, %xmm2
	vfnmadd132sd	%xmm5, %xmm2, %xmm1
	vfmsub132sd	%xmm5, %xmm1, %xmm0
	vxorpd	%xmm1, %xmm1, %xmm1
	vucomisd	%xmm1, %xmm0
	jp	.L449
	movq	%rbx, %rdx
	je	.L397
.L449:
	cmpq	%rbp, %rbx
	je	.L399
	vmovsd	%xmm0, (%rbx)
	leaq	8(%rbx), %rdx
.L397:
	cmpq	$8, %r14
	jbe	.L401
.L400:
	movl	$1, %ecx
	.p2align 4,,10
	.p2align 3
.L420:
	vmovsd	(%r8,%rcx,8), %xmm0
	vmovsd	.LC7(%rip), %xmm2
	vmovapd	%xmm4, %xmm7
	vmulsd	%xmm0, %xmm6, %xmm1
	vfmsub132sd	%xmm0, %xmm0, %xmm2
	vfmsub231sd	.LC7(%rip), %xmm0, %xmm2
	vfnmadd132sd	%xmm2, %xmm1, %xmm7
	vsubsd	%xmm2, %xmm0, %xmm0
	vfnmadd231sd	%xmm0, %xmm4, %xmm7
	vfnmadd132sd	%xmm5, %xmm7, %xmm2
	vfmsub132sd	%xmm5, %xmm2, %xmm0
	vaddsd	%xmm3, %xmm0, %xmm2
	vsubsd	%xmm3, %xmm2, %xmm8
	vsubsd	%xmm8, %xmm2, %xmm7
	vsubsd	%xmm8, %xmm0, %xmm0
	vsubsd	%xmm7, %xmm3, %xmm3
	vaddsd	%xmm0, %xmm3, %xmm0
	vxorpd	%xmm3, %xmm3, %xmm3
	vucomisd	%xmm3, %xmm0
	jp	.L451
	je	.L406
.L451:
	cmpq	%rbp, %rdx
	je	.L408
	vmovsd	%xmm0, (%rdx)
	addq	$8, %rdx
.L406:
	vaddsd	%xmm2, %xmm1, %xmm3
	vsubsd	%xmm1, %xmm3, %xmm0
	vsubsd	%xmm0, %xmm3, %xmm7
	vsubsd	%xmm0, %xmm2, %xmm2
	vsubsd	%xmm7, %xmm1, %xmm1
	vxorpd	%xmm7, %xmm7, %xmm7
	vaddsd	%xmm2, %xmm1, %xmm1
	vucomisd	%xmm7, %xmm1
	jp	.L452
	je	.L413
.L452:
	cmpq	%rbp, %rdx
	je	.L415
	vmovsd	%xmm1, (%rdx)
	addq	$8, %rdx
.L413:
	incq	%rcx
	cmpq	%r15, %rcx
	jb	.L420
.L401:
	vxorpd	%xmm5, %xmm5, %xmm5
	vucomisd	%xmm5, %xmm3
	jp	.L421
	je	.L500
.L421:
	cmpq	%rbp, %rdx
	je	.L403
	vmovsd	%xmm3, (%rdx)
	addq	$8, %rdx
.L500:
	movq	%rbp, %r14
	subq	%rbx, %r14
.L393:
	vmovq	%rbx, %xmm6
	leaq	128(%rsp), %rdi
	movq	%r12, %rsi
	movq	%rbp, 112(%rsp)
	vpinsrq	$1, %rdx, %xmm6, %xmm0
	leaq	96(%rsp), %rdx
	vmovdqa	%xmm0, 96(%rsp)
.LEHB31:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE31:
	movq	(%r12), %rdi
	vmovdqa	128(%rsp), %xmm0
	movq	144(%rsp), %rax
	movq	16(%r12), %rsi
	vmovdqu	%xmm0, (%r12)
	movq	%rax, 16(%r12)
	testq	%rdi, %rdi
	je	.L431
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L431:
	testq	%rbx, %rbx
	je	.L432
	movq	%r14, %rsi
	movq	%rbx, %rdi
	addq	$8, %r13
	call	_ZdlPvm@PLT
	cmpq	%r13, 16(%rsp)
	jne	.L435
.L391:
	movq	152(%rsp), %rax
	subq	%fs:40, %rax
	jne	.L501
	addq	$168, %rsp
	.cfi_remember_state
	.cfi_def_cfa_offset 56
	movq	%r12, %rax
	popq	%rbx
	.cfi_def_cfa_offset 48
	popq	%rbp
	.cfi_def_cfa_offset 40
	popq	%r12
	.cfi_def_cfa_offset 32
	popq	%r13
	.cfi_def_cfa_offset 24
	popq	%r14
	.cfi_def_cfa_offset 16
	popq	%r15
	.cfi_def_cfa_offset 8
	ret
	.p2align 4,,10
	.p2align 3
.L441:
	.cfi_restore_state
	xorl	%r14d, %r14d
	xorl	%ebp, %ebp
	xorl	%edx, %edx
	xorl	%ebx, %ebx
	jmp	.L393
	.p2align 4,,10
	.p2align 3
.L432:
	addq	$8, %r13
	cmpq	%r13, 16(%rsp)
	jne	.L435
	jmp	.L391
	.p2align 4,,10
	.p2align 3
.L415:
	subq	%rbx, %rdx
	movabsq	$1152921504606846975, %rsi
	movq	%rdx, %rax
	sarq	$3, %rax
	cmpq	%rsi, %rax
	je	.L494
	testq	%rax, %rax
	movl	$1, %r14d
	movq	%rdx, 80(%rsp)
	cmovne	%rax, %r14
	movq	%rcx, 72(%rsp)
	movq	%r8, 32(%rsp)
	addq	%rax, %r14
	movabsq	$1152921504606846975, %rax
	vmovsd	%xmm1, 64(%rsp)
	cmpq	%rax, %r14
	vmovsd	%xmm3, 56(%rsp)
	cmova	%rax, %r14
	vmovsd	%xmm5, 48(%rsp)
	vmovsd	%xmm4, 40(%rsp)
	salq	$3, %r14
	vmovsd	%xmm6, 24(%rsp)
	movq	%r14, %rdi
.LEHB32:
	call	_Znwm@PLT
	movq	80(%rsp), %rdx
	vmovsd	64(%rsp), %xmm1
	movq	%rax, %r10
	vmovsd	24(%rsp), %xmm6
	movq	32(%rsp), %r8
	testq	%rdx, %rdx
	vmovsd	40(%rsp), %xmm4
	vmovsd	48(%rsp), %xmm5
	vmovsd	%xmm1, (%rax,%rdx)
	vmovsd	56(%rsp), %xmm3
	movq	72(%rsp), %rcx
	je	.L418
	movq	%rbx, %rsi
	movq	%rax, %rdi
	movq	%r8, 40(%rsp)
	movq	%rdx, 24(%rsp)
	vmovsd	%xmm3, 64(%rsp)
	vmovsd	%xmm5, 56(%rsp)
	vmovsd	%xmm4, 48(%rsp)
	vmovsd	%xmm6, 32(%rsp)
	call	memcpy@PLT
	movq	72(%rsp), %rcx
	vmovsd	64(%rsp), %xmm3
	vmovsd	56(%rsp), %xmm5
	vmovsd	48(%rsp), %xmm4
	movq	%rax, %r10
	movq	40(%rsp), %r8
	vmovsd	32(%rsp), %xmm6
	movq	24(%rsp), %rdx
.L418:
	leaq	8(%r10,%rdx), %rdx
	testq	%rbx, %rbx
	je	.L419
	movq	%rbp, %rsi
	movq	%rbx, %rdi
	movq	%r10, 80(%rsp)
	subq	%rbx, %rsi
	movq	%rdx, 72(%rsp)
	movq	%rcx, 64(%rsp)
	movq	%r8, 32(%rsp)
	vmovsd	%xmm3, 56(%rsp)
	vmovsd	%xmm5, 48(%rsp)
	vmovsd	%xmm4, 40(%rsp)
	vmovsd	%xmm6, 24(%rsp)
	call	_ZdlPvm@PLT
	movq	80(%rsp), %r10
	movq	72(%rsp), %rdx
	movq	64(%rsp), %rcx
	vmovsd	56(%rsp), %xmm3
	vmovsd	48(%rsp), %xmm5
	vmovsd	40(%rsp), %xmm4
	movq	32(%rsp), %r8
	vmovsd	24(%rsp), %xmm6
.L419:
	leaq	(%r10,%r14), %rbp
	movq	%r10, %rbx
	jmp	.L413
	.p2align 4,,10
	.p2align 3
.L408:
	subq	%rbx, %rdx
	movabsq	$1152921504606846975, %rsi
	movq	%rdx, %rax
	sarq	$3, %rax
	cmpq	%rsi, %rax
	je	.L493
	testq	%rax, %rax
	movl	$1, %r14d
	movq	%rdx, 88(%rsp)
	cmovne	%rax, %r14
	movq	%rcx, 80(%rsp)
	movq	%r8, 32(%rsp)
	addq	%rax, %r14
	movabsq	$1152921504606846975, %rax
	vmovsd	%xmm0, 72(%rsp)
	cmpq	%rax, %r14
	vmovsd	%xmm2, 64(%rsp)
	cmova	%rax, %r14
	vmovsd	%xmm1, 56(%rsp)
	vmovsd	%xmm5, 48(%rsp)
	salq	$3, %r14
	vmovsd	%xmm4, 40(%rsp)
	movq	%r14, %rdi
	vmovsd	%xmm6, 24(%rsp)
	call	_Znwm@PLT
	movq	88(%rsp), %rdx
	vmovsd	72(%rsp), %xmm0
	movq	%rax, %r10
	vmovsd	24(%rsp), %xmm6
	movq	32(%rsp), %r8
	testq	%rdx, %rdx
	vmovsd	40(%rsp), %xmm4
	vmovsd	48(%rsp), %xmm5
	vmovsd	%xmm0, (%rax,%rdx)
	vmovsd	56(%rsp), %xmm1
	vmovsd	64(%rsp), %xmm2
	movq	80(%rsp), %rcx
	je	.L411
	movq	%rbx, %rsi
	movq	%rax, %rdi
	movq	%r8, 40(%rsp)
	movq	%rdx, 24(%rsp)
	vmovsd	%xmm2, 72(%rsp)
	vmovsd	%xmm1, 64(%rsp)
	vmovsd	%xmm5, 56(%rsp)
	vmovsd	%xmm4, 48(%rsp)
	vmovsd	%xmm6, 32(%rsp)
	call	memcpy@PLT
	movq	80(%rsp), %rcx
	vmovsd	72(%rsp), %xmm2
	vmovsd	64(%rsp), %xmm1
	vmovsd	56(%rsp), %xmm5
	movq	%rax, %r10
	vmovsd	48(%rsp), %xmm4
	movq	40(%rsp), %r8
	vmovsd	32(%rsp), %xmm6
	movq	24(%rsp), %rdx
.L411:
	leaq	8(%r10,%rdx), %rdx
	testq	%rbx, %rbx
	je	.L412
	movq	%rbp, %rsi
	movq	%rbx, %rdi
	movq	%r10, 88(%rsp)
	subq	%rbx, %rsi
	movq	%rdx, 80(%rsp)
	movq	%rcx, 72(%rsp)
	movq	%r8, 32(%rsp)
	vmovsd	%xmm2, 64(%rsp)
	vmovsd	%xmm1, 56(%rsp)
	vmovsd	%xmm5, 48(%rsp)
	vmovsd	%xmm4, 40(%rsp)
	vmovsd	%xmm6, 24(%rsp)
	call	_ZdlPvm@PLT
	movq	88(%rsp), %r10
	movq	80(%rsp), %rdx
	movq	72(%rsp), %rcx
	vmovsd	64(%rsp), %xmm2
	vmovsd	56(%rsp), %xmm1
	vmovsd	48(%rsp), %xmm5
	vmovsd	40(%rsp), %xmm4
	movq	32(%rsp), %r8
	vmovsd	24(%rsp), %xmm6
.L412:
	leaq	(%r10,%r14), %rbp
	movq	%r10, %rbx
	jmp	.L406
	.p2align 4,,10
	.p2align 3
.L442:
	xorl	%ebp, %ebp
	xorl	%ebx, %ebx
	jmp	.L396
	.p2align 4,,10
	.p2align 3
.L399:
	movl	$8, %edi
	movq	%r8, 32(%rsp)
	vmovsd	%xmm3, 64(%rsp)
	vmovsd	%xmm0, 56(%rsp)
	vmovsd	%xmm5, 48(%rsp)
	vmovsd	%xmm4, 40(%rsp)
	vmovsd	%xmm6, 24(%rsp)
	call	_Znwm@PLT
	vmovsd	56(%rsp), %xmm0
	testq	%rbx, %rbx
	vmovsd	24(%rsp), %xmm6
	leaq	8(%rax), %rdx
	movq	32(%rsp), %r8
	vmovsd	40(%rsp), %xmm4
	vmovsd	48(%rsp), %xmm5
	vmovsd	64(%rsp), %xmm3
	vmovsd	%xmm0, (%rax)
	je	.L402
	xorl	%esi, %esi
	movq	%rbx, %rdi
	movq	%rax, 72(%rsp)
	movq	%rdx, 64(%rsp)
	vmovsd	%xmm3, 56(%rsp)
	call	_ZdlPvm@PLT
	movq	72(%rsp), %rax
	movq	64(%rsp), %rdx
	vmovsd	56(%rsp), %xmm3
	vmovsd	48(%rsp), %xmm5
	vmovsd	40(%rsp), %xmm4
	movq	32(%rsp), %r8
	vmovsd	24(%rsp), %xmm6
.L402:
	movq	%rdx, %rbp
	movq	%rax, %rbx
	cmpq	$8, %r14
	ja	.L400
	vxorpd	%xmm0, %xmm0, %xmm0
	vucomisd	%xmm0, %xmm3
	jp	.L445
	je	.L500
.L445:
	movq	%rdx, %rbp
	movq	%rax, %rbx
	.p2align 4,,10
	.p2align 3
.L403:
	subq	%rbx, %rdx
	movabsq	$1152921504606846975, %rcx
	movq	%rdx, %rax
	sarq	$3, %rax
	cmpq	%rcx, %rax
	je	.L495
	testq	%rax, %rax
	movl	$1, %r14d
	movq	%rdx, 32(%rsp)
	cmovne	%rax, %r14
	vmovsd	%xmm3, 24(%rsp)
	addq	%rax, %r14
	movabsq	$1152921504606846975, %rax
	cmpq	%rax, %r14
	cmova	%rax, %r14
	salq	$3, %r14
	movq	%r14, %rdi
	call	_Znwm@PLT
.LEHE32:
	vmovsd	24(%rsp), %xmm3
	movq	32(%rsp), %rdx
	movq	%rax, %rcx
	vmovsd	%xmm3, (%rax,%rdx)
	testq	%rdx, %rdx
	jne	.L502
	leaq	8(%rcx,%rdx), %rdx
	testq	%rbx, %rbx
	je	.L426
.L503:
	movq	%rbp, %rsi
	movq	%rbx, %rdi
	movq	%rcx, 32(%rsp)
	subq	%rbx, %rsi
	movq	%rdx, 24(%rsp)
	call	_ZdlPvm@PLT
	movq	32(%rsp), %rcx
	movq	24(%rsp), %rdx
.L426:
	leaq	(%rcx,%r14), %rbp
	movq	%rcx, %rbx
	jmp	.L393
.L502:
	movq	%rbx, %rsi
	movq	%rax, %rdi
	movq	%rdx, 24(%rsp)
	call	memcpy@PLT
	movq	24(%rsp), %rdx
	movq	%rax, %rcx
	leaq	8(%rcx,%rdx), %rdx
	testq	%rbx, %rbx
	jne	.L503
	jmp	.L426
.L501:
	call	__stack_chk_fail@PLT
.L446:
	endbr64
	movq	%rax, %rbp
	jmp	.L436
.L488:
	endbr64
	jmp	.L489
.L490:
	endbr64
	jmp	.L491
	.section	.gcc_except_table
.LLSDA2296:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSE2296-.LLSDACSB2296
.LLSDACSB2296:
	.uleb128 .LEHB30-.LFB2296
	.uleb128 .LEHE30-.LEHB30
	.uleb128 .L488-.LFB2296
	.uleb128 0
	.uleb128 .LEHB31-.LFB2296
	.uleb128 .LEHE31-.LEHB31
	.uleb128 .L446-.LFB2296
	.uleb128 0
	.uleb128 .LEHB32-.LFB2296
	.uleb128 .LEHE32-.LEHB32
	.uleb128 .L490-.LFB2296
	.uleb128 0
.LLSDACSE2296:
	.text
	.cfi_endproc
	.section	.text.unlikely
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDAC2296
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_.cold, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_.cold:
.LFSB2296:
.L494:
	.cfi_def_cfa_offset 224
	.cfi_offset 3, -56
	.cfi_offset 6, -48
	.cfi_offset 12, -40
	.cfi_offset 13, -32
	.cfi_offset 14, -24
	.cfi_offset 15, -16
	movq	152(%rsp), %rax
	subq	%fs:40, %rax
	jne	.L504
	leaq	.LC1(%rip), %rdi
.LEHB33:
	call	_ZSt20__throw_length_errorPKc@PLT
.LEHE33:
.L436:
	testq	%rbx, %rbx
	je	.L498
	movq	%r14, %rsi
	movq	%rbx, %rdi
	vzeroupper
	call	_ZdlPvm@PLT
	jmp	.L430
.L447:
	endbr64
.L489:
	movq	%rax, %rbp
	vzeroupper
.L430:
	movq	(%r12), %rdi
	testq	%rdi, %rdi
	je	.L438
	movq	16(%r12), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L438:
	movq	152(%rsp), %rax
	subq	%fs:40, %rax
	jne	.L505
	movq	%rbp, %rdi
.LEHB34:
	call	_Unwind_Resume@PLT
.LEHE34:
.L498:
	vzeroupper
	jmp	.L430
.L492:
	movq	152(%rsp), %rax
	subq	%fs:40, %rax
	jne	.L506
	leaq	.LC0(%rip), %rdi
.LEHB35:
	call	_ZSt20__throw_length_errorPKc@PLT
.LEHE35:
.L493:
	movq	152(%rsp), %rax
	subq	%fs:40, %rax
	jne	.L507
	leaq	.LC1(%rip), %rdi
.LEHB36:
	call	_ZSt20__throw_length_errorPKc@PLT
.L506:
	call	__stack_chk_fail@PLT
.L504:
	call	__stack_chk_fail@PLT
.L495:
	movq	152(%rsp), %rax
	subq	%fs:40, %rax
	jne	.L508
	leaq	.LC1(%rip), %rdi
	call	_ZSt20__throw_length_errorPKc@PLT
.LEHE36:
.L508:
	call	__stack_chk_fail@PLT
.L505:
	call	__stack_chk_fail@PLT
.L448:
	endbr64
.L491:
	movq	%rax, %r13
	testq	%rbx, %rbx
	je	.L497
	movq	%rbp, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
.L429:
	movq	%r13, %rbp
	jmp	.L430
.L507:
	call	__stack_chk_fail@PLT
.L497:
	vzeroupper
	jmp	.L429
	.cfi_endproc
.LFE2296:
	.section	.gcc_except_table
.LLSDAC2296:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSEC2296-.LLSDACSBC2296
.LLSDACSBC2296:
	.uleb128 .LEHB33-.LCOLDB17
	.uleb128 .LEHE33-.LEHB33
	.uleb128 .L448-.LCOLDB17
	.uleb128 0
	.uleb128 .LEHB34-.LCOLDB17
	.uleb128 .LEHE34-.LEHB34
	.uleb128 0
	.uleb128 0
	.uleb128 .LEHB35-.LCOLDB17
	.uleb128 .LEHE35-.LEHB35
	.uleb128 .L447-.LCOLDB17
	.uleb128 0
	.uleb128 .LEHB36-.LCOLDB17
	.uleb128 .LEHE36-.LEHB36
	.uleb128 .L448-.LCOLDB17
	.uleb128 0
.LLSDACSEC2296:
	.section	.text.unlikely
	.text
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
	.section	.text.unlikely
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_.cold, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_.cold
.LCOLDE17:
	.text
.LHOTE17:
	.section	.text.unlikely
.LCOLDB18:
	.text
.LHOTB18:
	.p2align 4
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13incircleExactENS0_4Vec2ES3_S3_S3_, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13incircleExactENS0_4Vec2ES3_S3_S3_:
.LFB2312:
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDA2312
	leaq	8(%rsp), %r10
	.cfi_def_cfa 10, 0
	andq	$-32, %rsp
	pushq	-8(%r10)
	pushq	%rbp
	movq	%rsp, %rbp
	.cfi_escape 0x10,0x6,0x2,0x76,0
	pushq	%r15
	pushq	%r14
	.cfi_escape 0x10,0xf,0x2,0x76,0x78
	.cfi_escape 0x10,0xe,0x2,0x76,0x70
	vmovq	%xmm5, %r14
	pushq	%r13
	.cfi_escape 0x10,0xd,0x2,0x76,0x68
	vmovq	%xmm1, %r13
	vmovapd	%xmm6, %xmm1
	pushq	%r12
	.cfi_escape 0x10,0xc,0x2,0x76,0x60
	vmovq	%xmm4, %r12
	pushq	%r10
	.cfi_escape 0xf,0x3,0x76,0x58,0x6
	pushq	%rbx
	.cfi_escape 0x10,0x3,0x2,0x76,0x50
	vmovq	%xmm7, %rbx
	subq	$576, %rsp
	vmovsd	%xmm2, -552(%rbp)
	vmovsd	%xmm3, -544(%rbp)
	movq	%fs:40, %rax
	movq	%rax, -56(%rbp)
	leaq	-528(%rbp), %rax
	vmovsd	%xmm6, -536(%rbp)
	movq	%rax, %rdi
	movq	%rax, -560(%rbp)
.LEHB37:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd
.LEHE37:
	leaq	-496(%rbp), %rcx
	vmovq	%r13, %xmm0
	vmovq	%rbx, %xmm1
	movq	%rcx, %rdi
	movq	%rcx, -592(%rbp)
	movq	%rcx, %r13
.LEHB38:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd
.LEHE38:
	leaq	-464(%rbp), %rcx
	vmovsd	-536(%rbp), %xmm1
	vmovsd	-552(%rbp), %xmm0
	movq	%rcx, %rdi
	movq	%rcx, -568(%rbp)
.LEHB39:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd
.LEHE39:
	leaq	-432(%rbp), %rcx
	vmovsd	-544(%rbp), %xmm0
	vmovq	%rbx, %xmm1
	movq	%rcx, %rdi
	movq	%rcx, -576(%rbp)
.LEHB40:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd
.LEHE40:
	leaq	-400(%rbp), %rcx
	vmovsd	-536(%rbp), %xmm1
	vmovq	%r12, %xmm0
	movq	%rcx, %rdi
	movq	%rcx, -584(%rbp)
.LEHB41:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd
.LEHE41:
	leaq	-368(%rbp), %r15
	vmovq	%rbx, %xmm1
	vmovq	%r14, %xmm0
	movq	%r15, %rdi
.LEHB42:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L11twoTermDiffEdd
.LEHE42:
	leaq	-80(%rbp), %rcx
	movq	%r13, %rdx
	movq	%r13, %rsi
	movq	%rcx, %rdi
	movq	%rcx, -544(%rbp)
.LEHB43:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE43:
	movq	-560(%rbp), %rsi
	leaq	-112(%rbp), %rcx
	movq	-80(%rbp), %rbx
	movq	%rcx, %rdi
	movq	%rcx, -536(%rbp)
	movq	-64(%rbp), %r13
	movq	%rsi, %rdx
.LEHB44:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE44:
	leaq	-336(%rbp), %rax
	movq	-112(%rbp), %r12
	movq	-96(%rbp), %r14
	movq	%rbx, -80(%rbp)
	movq	-544(%rbp), %rdx
	movq	-536(%rbp), %rsi
	movq	%rax, %rdi
	movq	%r13, -64(%rbp)
	movq	%r12, -112(%rbp)
	movq	%r14, -96(%rbp)
	movq	%rax, -600(%rbp)
.LEHB45:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE45:
	testq	%r12, %r12
	je	.L510
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L510:
	testq	%rbx, %rbx
	je	.L511
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L511:
	movq	-576(%rbp), %rdx
	movq	-544(%rbp), %rdi
	movq	%rdx, %rsi
.LEHB46:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE46:
	movq	-568(%rbp), %rsi
	movq	-536(%rbp), %rdi
	movq	-80(%rbp), %rbx
	movq	-64(%rbp), %r13
	movq	%rsi, %rdx
.LEHB47:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE47:
	leaq	-304(%rbp), %rax
	movq	-112(%rbp), %r12
	movq	-96(%rbp), %r14
	movq	%rbx, -80(%rbp)
	movq	-544(%rbp), %rdx
	movq	-536(%rbp), %rsi
	movq	%rax, %rdi
	movq	%r13, -64(%rbp)
	movq	%r12, -112(%rbp)
	movq	%r14, -96(%rbp)
	movq	%rax, -608(%rbp)
.LEHB48:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE48:
	testq	%r12, %r12
	je	.L512
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L512:
	testq	%rbx, %rbx
	je	.L513
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L513:
	movq	-544(%rbp), %rdi
	movq	%r15, %rdx
	movq	%r15, %rsi
.LEHB49:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE49:
	movq	-584(%rbp), %rsi
	movq	-536(%rbp), %rdi
	movq	-80(%rbp), %rbx
	movq	-64(%rbp), %r13
	movq	%rsi, %rdx
.LEHB50:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE50:
	leaq	-272(%rbp), %rax
	movq	-112(%rbp), %r12
	movq	-96(%rbp), %r14
	movq	%rbx, -80(%rbp)
	movq	-544(%rbp), %rdx
	movq	-536(%rbp), %rsi
	movq	%rax, %rdi
	movq	%r13, -64(%rbp)
	movq	%r12, -112(%rbp)
	movq	%r14, -96(%rbp)
	movq	%rax, -616(%rbp)
.LEHB51:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE51:
	testq	%r12, %r12
	je	.L514
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L514:
	testq	%rbx, %rbx
	je	.L515
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L515:
	movq	-576(%rbp), %rdx
	movq	-584(%rbp), %rsi
	movq	-536(%rbp), %rdi
.LEHB52:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE52:
	vmovdqa	-112(%rbp), %xmm2
	vpextrq	$1, %xmm2, %rax
	vmovq	%xmm2, %rbx
	cmpq	%rax, %rbx
	je	.L516
	subq	$8, %rax
	subq	%rbx, %rax
	movq	%rax, %rsi
	shrq	$3, %rsi
	leaq	1(%rsi), %rcx
	cmpq	$16, %rax
	jbe	.L625
	movq	%rcx, %rdx
	vbroadcastsd	.LC10(%rip), %ymm0
	movq	%rbx, %rax
	shrq	$2, %rdx
	salq	$5, %rdx
	addq	%rbx, %rdx
	.p2align 5
	.p2align 4,,10
	.p2align 3
.L518:
	vxorpd	(%rax), %ymm0, %ymm1
	addq	$32, %rax
	vmovupd	%ymm1, -32(%rax)
	cmpq	%rdx, %rax
	jne	.L518
	testb	$3, %cl
	je	.L874
	movq	%rcx, %rax
	andq	$-4, %rax
	leaq	(%rbx,%rax,8), %rdx
	vzeroupper
.L517:
	cmpq	%rax, %rsi
	je	.L520
	vmovddup	.LC10(%rip), %xmm0
	movq	%rcx, %rsi
	subq	%rax, %rsi
	andl	$1, %ecx
	vxorpd	(%rbx,%rax,8), %xmm0, %xmm0
	vmovupd	%xmm0, (%rbx,%rax,8)
	je	.L516
	andq	$-2, %rsi
	leaq	(%rdx,%rsi,8), %rdx
.L520:
	vmovsd	(%rdx), %xmm0
	vxorpd	.LC12(%rip), %xmm0, %xmm0
	vmovsd	%xmm0, (%rdx)
.L516:
	leaq	-144(%rbp), %rax
	movq	-96(%rbp), %r13
	movq	-568(%rbp), %rsi
	movq	%r15, %rdx
	movq	%rax, %rdi
	movq	%rax, -552(%rbp)
	movq	%r13, -64(%rbp)
	vmovdqa	%xmm2, -80(%rbp)
.LEHB53:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE53:
	leaq	-240(%rbp), %rax
	movq	-144(%rbp), %r12
	movq	-128(%rbp), %r14
	movq	-544(%rbp), %rdx
	movq	-552(%rbp), %rsi
	movq	%rax, %rdi
	movq	%rax, -624(%rbp)
	movq	%r12, -144(%rbp)
	movq	%r14, -128(%rbp)
.LEHB54:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE54:
	testq	%r12, %r12
	je	.L522
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L522:
	testq	%rbx, %rbx
	je	.L523
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L523:
	movq	-560(%rbp), %rsi
	movq	-536(%rbp), %rdi
	movq	%r15, %rdx
.LEHB55:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE55:
	vmovdqa	-112(%rbp), %xmm2
	vpextrq	$1, %xmm2, %rax
	vmovq	%xmm2, %rbx
	cmpq	%rax, %rbx
	je	.L524
	subq	$8, %rax
	subq	%rbx, %rax
	movq	%rax, %rsi
	shrq	$3, %rsi
	leaq	1(%rsi), %rcx
	cmpq	$16, %rax
	jbe	.L626
	movq	%rcx, %rdx
	vbroadcastsd	.LC10(%rip), %ymm0
	movq	%rbx, %rax
	shrq	$2, %rdx
	salq	$5, %rdx
	addq	%rbx, %rdx
	.p2align 5
	.p2align 4,,10
	.p2align 3
.L526:
	vxorpd	(%rax), %ymm0, %ymm1
	addq	$32, %rax
	vmovupd	%ymm1, -32(%rax)
	cmpq	%rdx, %rax
	jne	.L526
	testb	$3, %cl
	je	.L875
	movq	%rcx, %rax
	andq	$-4, %rax
	leaq	(%rbx,%rax,8), %rdx
	vzeroupper
.L525:
	cmpq	%rax, %rsi
	je	.L528
	vmovddup	.LC10(%rip), %xmm0
	movq	%rcx, %rsi
	subq	%rax, %rsi
	andl	$1, %ecx
	vxorpd	(%rbx,%rax,8), %xmm0, %xmm0
	vmovupd	%xmm0, (%rbx,%rax,8)
	je	.L524
	andq	$-2, %rsi
	leaq	(%rdx,%rsi,8), %rdx
.L528:
	vmovsd	(%rdx), %xmm0
	vxorpd	.LC12(%rip), %xmm0, %xmm0
	vmovsd	%xmm0, (%rdx)
.L524:
	movq	-552(%rbp), %r15
	movq	-96(%rbp), %r13
	vmovdqa	%xmm2, -80(%rbp)
	movq	-592(%rbp), %rdx
	movq	-584(%rbp), %rsi
	movq	%r15, %rdi
	movq	%r13, -64(%rbp)
.LEHB56:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE56:
	leaq	-208(%rbp), %rax
	movq	-144(%rbp), %r12
	movq	-128(%rbp), %r14
	movq	%r15, %rsi
	movq	-544(%rbp), %rdx
	movq	%rax, %rdi
	movq	%rax, -584(%rbp)
	movq	%r12, -144(%rbp)
	movq	%r14, -128(%rbp)
.LEHB57:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE57:
	testq	%r12, %r12
	je	.L530
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L530:
	testq	%rbx, %rbx
	je	.L531
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L531:
	movq	-592(%rbp), %rdx
	movq	-568(%rbp), %rsi
	movq	-536(%rbp), %rdi
.LEHB58:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE58:
	vmovdqa	-112(%rbp), %xmm2
	vpextrq	$1, %xmm2, %rax
	vmovq	%xmm2, %rbx
	cmpq	%rax, %rbx
	je	.L532
	subq	$8, %rax
	subq	%rbx, %rax
	movq	%rax, %rsi
	shrq	$3, %rsi
	leaq	1(%rsi), %rcx
	cmpq	$16, %rax
	jbe	.L627
	movq	%rcx, %rdx
	vbroadcastsd	.LC10(%rip), %ymm0
	movq	%rbx, %rax
	shrq	$2, %rdx
	salq	$5, %rdx
	addq	%rbx, %rdx
	.p2align 5
	.p2align 4,,10
	.p2align 3
.L534:
	vxorpd	(%rax), %ymm0, %ymm1
	addq	$32, %rax
	vmovupd	%ymm1, -32(%rax)
	cmpq	%rdx, %rax
	jne	.L534
	testb	$3, %cl
	je	.L876
	movq	%rcx, %rax
	andq	$-4, %rax
	leaq	(%rbx,%rax,8), %rdx
	vzeroupper
.L533:
	cmpq	%rax, %rsi
	je	.L536
	vmovddup	.LC10(%rip), %xmm0
	movq	%rcx, %rsi
	subq	%rax, %rsi
	andl	$1, %ecx
	vxorpd	(%rbx,%rax,8), %xmm0, %xmm0
	vmovupd	%xmm0, (%rbx,%rax,8)
	je	.L532
	andq	$-2, %rsi
	leaq	(%rdx,%rsi,8), %rdx
.L536:
	vmovsd	(%rdx), %xmm0
	vxorpd	.LC12(%rip), %xmm0, %xmm0
	vmovsd	%xmm0, (%rdx)
.L532:
	movq	-552(%rbp), %r15
	movq	-96(%rbp), %r13
	vmovdqa	%xmm2, -80(%rbp)
	movq	-576(%rbp), %rdx
	movq	-560(%rbp), %rsi
	movq	%r15, %rdi
	movq	%r13, -64(%rbp)
.LEHB59:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE59:
	leaq	-176(%rbp), %rax
	movq	-144(%rbp), %r12
	movq	-128(%rbp), %r14
	movq	%r15, %rsi
	movq	-544(%rbp), %rdx
	movq	%rax, %rdi
	movq	%rax, -560(%rbp)
	movq	%r12, -144(%rbp)
	movq	%r14, -128(%rbp)
.LEHB60:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE60:
	testq	%r12, %r12
	je	.L538
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L538:
	testq	%rbx, %rbx
	je	.L539
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L539:
	movq	-624(%rbp), %rdx
	movq	-600(%rbp), %rsi
	movq	-552(%rbp), %rdi
.LEHB61:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE61:
	movq	-584(%rbp), %rdx
	movq	-608(%rbp), %rsi
	movq	-536(%rbp), %rdi
	movq	-144(%rbp), %r13
	movq	-128(%rbp), %r15
.LEHB62:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE62:
	movq	-96(%rbp), %rax
	movq	-112(%rbp), %r12
	movq	%r13, -144(%rbp)
	movq	-536(%rbp), %rdx
	movq	-552(%rbp), %rsi
	movq	%r15, -128(%rbp)
	movq	-544(%rbp), %rdi
	movq	%rax, -568(%rbp)
	movq	%r12, -112(%rbp)
	movq	%rax, -96(%rbp)
.LEHB63:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE63:
	movq	-72(%rbp), %rax
	movq	-80(%rbp), %rbx
	movq	-64(%rbp), %r14
	movq	%rax, -136(%rbp)
	testq	%r13, %r13
	je	.L543
	movq	%r15, %rsi
	movq	%r13, %rdi
	subq	%r13, %rsi
	call	_ZdlPvm@PLT
.L543:
	testq	%r12, %r12
	je	.L544
	movq	-568(%rbp), %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	call	_ZdlPvm@PLT
.L544:
	movq	-536(%rbp), %r12
	movq	-560(%rbp), %rdx
	movq	-616(%rbp), %rsi
	movq	%r12, %rdi
.LEHB64:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L16expansionProductERKSt6vectorIdSaIdEES7_
.LEHE64:
	movq	-112(%rbp), %r13
	movq	-96(%rbp), %r15
	movq	%r12, %rdx
	movq	%rbx, -144(%rbp)
	movq	-552(%rbp), %rsi
	movq	-544(%rbp), %rdi
	movq	%r14, -128(%rbp)
	movq	%r13, -112(%rbp)
	movq	%r15, -96(%rbp)
.LEHB65:
	call	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L12expansionSumERKSt6vectorIdSaIdEES7_
.LEHE65:
	movq	-64(%rbp), %rax
	movq	-80(%rbp), %r8
	movq	-72(%rbp), %r12
	movq	%rax, -536(%rbp)
	testq	%rbx, %rbx
	je	.L545
	movq	%r14, %rsi
	movq	%rbx, %rdi
	movq	%r8, -544(%rbp)
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
	movq	-544(%rbp), %r8
.L545:
	testq	%r13, %r13
	je	.L546
	movq	%r15, %rsi
	movq	%r13, %rdi
	movq	%r8, -544(%rbp)
	subq	%r13, %rsi
	call	_ZdlPvm@PLT
	movq	-544(%rbp), %r8
.L546:
	subq	%r8, %r12
	movq	%r12, %rcx
	sarq	$3, %rcx
	movq	%rcx, %rdx
	cmpq	$40, %r12
	jbe	.L547
	leaq	-32(%r8,%r12), %rax
	testb	$31, %al
	jne	.L547
	vmovq	%rcx, %xmm5
	incq	%rcx
	vpbroadcastq	.LC15(%rip), %ymm4
	xorl	%edx, %edx
	vpbroadcastq	%xmm5, %ymm1
	shrq	$2, %rcx
	vxorpd	%xmm3, %xmm3, %xmm3
	vpaddq	.LC14(%rip), %ymm1, %ymm1
	decq	%rcx
	jmp	.L550
	.p2align 4,,10
	.p2align 3
.L555:
	vpermpd	$27, (%rax), %ymm0
	vcmpltpd	%ymm0, %ymm3, %ymm2
	vptest	%ymm2, %ymm2
	jne	.L873
	vcmpltpd	%ymm3, %ymm0, %ymm0
	subq	$32, %rax
	incq	%rdx
	vptest	%ymm0, %ymm0
	jne	.L873
	vpaddq	%ymm4, %ymm1, %ymm1
.L550:
	cmpq	%rdx, %rcx
	jne	.L555
.L873:
	vmovq	%xmm1, %rax
	testq	%rax, %rax
	je	.L650
	leaq	-1(%rax), %rdx
	vxorpd	%xmm0, %xmm0, %xmm0
	vmovsd	(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L648
	vcomisd	%xmm1, %xmm0
	ja	.L649
	testq	%rdx, %rdx
	je	.L650
	vmovsd	-8(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L648
	vcomisd	%xmm1, %xmm0
	ja	.L649
	cmpq	$2, %rax
	je	.L650
	vmovsd	-16(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L648
	vcomisd	%xmm1, %xmm0
	ja	.L649
	cmpq	$3, %rax
	je	.L650
	vmovsd	-24(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L648
	vcomisd	%xmm1, %xmm0
	ja	.L649
	cmpq	$4, %rax
	je	.L650
	vmovsd	-32(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L648
	vcomisd	%xmm1, %xmm0
	ja	.L649
	cmpq	$5, %rax
	je	.L650
	vmovsd	-40(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L648
	vcomisd	%xmm1, %xmm0
	ja	.L649
	cmpq	$6, %rax
	je	.L650
	vmovsd	-48(%r8,%rdx,8), %xmm1
	vcomisd	%xmm0, %xmm1
	ja	.L648
	vcomisd	%xmm1, %xmm0
	ja	.L649
	cmpq	$7, %rax
	je	.L650
	vmovsd	-56(%r8,%rdx,8), %xmm1
	xorl	%ebx, %ebx
	vcomisd	%xmm0, %xmm1
	seta	%bl
	leal	-1(%rbx,%rbx), %ebx
	vzeroupper
.L552:
	testq	%r8, %r8
	jne	.L553
.L557:
	movq	-176(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L558
	movq	-160(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L558:
	movq	-208(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L559
	movq	-192(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L559:
	movq	-240(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L560
	movq	-224(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L560:
	movq	-272(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L561
	movq	-256(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L561:
	movq	-304(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L562
	movq	-288(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L562:
	movq	-336(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L563
	movq	-320(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L563:
	movq	-368(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L564
	movq	-352(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L564:
	movq	-400(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L565
	movq	-384(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L565:
	movq	-432(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L566
	movq	-416(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L566:
	movq	-464(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L567
	movq	-448(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L567:
	movq	-496(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L568
	movq	-480(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L568:
	movq	-528(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L509
	movq	-512(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L509:
	movq	-56(%rbp), %rax
	subq	%fs:40, %rax
	jne	.L886
	addq	$576, %rsp
	movl	%ebx, %eax
	popq	%rbx
	popq	%r10
	.cfi_remember_state
	.cfi_def_cfa 10, 0
	popq	%r12
	popq	%r13
	popq	%r14
	popq	%r15
	popq	%rbp
	leaq	-8(%r10), %rsp
	.cfi_def_cfa 7, 8
	ret
.L653:
	.cfi_restore_state
	movl	$1, %ebx
	.p2align 4,,10
	.p2align 3
.L553:
	movq	-536(%rbp), %rsi
	movq	%r8, %rdi
	subq	%r8, %rsi
	call	_ZdlPvm@PLT
	jmp	.L557
	.p2align 4,,10
	.p2align 3
.L876:
	vzeroupper
	jmp	.L532
	.p2align 4,,10
	.p2align 3
.L874:
	vzeroupper
	jmp	.L516
	.p2align 4,,10
	.p2align 3
.L875:
	vzeroupper
	jmp	.L524
	.p2align 4,,10
	.p2align 3
.L650:
	xorl	%ebx, %ebx
	vzeroupper
	jmp	.L552
	.p2align 4,,10
	.p2align 3
.L547:
	vxorpd	%xmm1, %xmm1, %xmm1
	jmp	.L554
	.p2align 4,,10
	.p2align 3
.L888:
	decq	%rdx
	vmovsd	(%r8,%rdx,8), %xmm0
	vcomisd	%xmm1, %xmm0
	ja	.L653
	vcomisd	%xmm0, %xmm1
	ja	.L887
.L554:
	testq	%rdx, %rdx
	jne	.L888
	xorl	%ebx, %ebx
	jmp	.L552
	.p2align 4,,10
	.p2align 3
.L648:
	movl	$1, %ebx
	vzeroupper
	jmp	.L552
	.p2align 4,,10
	.p2align 3
.L649:
	movl	$-1, %ebx
	vzeroupper
	jmp	.L552
.L626:
	movq	%rbx, %rdx
	xorl	%eax, %eax
	jmp	.L525
.L625:
	movq	%rbx, %rdx
	xorl	%eax, %eax
	jmp	.L517
.L627:
	movq	%rbx, %rdx
	xorl	%eax, %eax
	jmp	.L533
.L887:
	movl	$-1, %ebx
	jmp	.L553
.L886:
	call	__stack_chk_fail@PLT
.L679:
	endbr64
	movq	%rax, %rbx
	jmp	.L600
.L682:
	endbr64
	movq	%rax, %r12
	jmp	.L541
.L677:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L599
.L671:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L589
.L667:
	endbr64
	movq	%rax, %r15
	jmp	.L580
.L666:
	endbr64
	movq	%rax, %r15
	vzeroupper
	jmp	.L582
.L663:
	endbr64
	movq	%rax, %r15
	vzeroupper
	jmp	.L577
.L665:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L579
.L661:
	endbr64
	movq	%rax, %r15
	jmp	.L570
.L660:
	endbr64
	movq	%rax, %r15
	vzeroupper
	jmp	.L572
.L668:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L584
.L664:
	endbr64
	movq	%rax, %r15
	jmp	.L575
.L662:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L574
.L659:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L613
.L658:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L615
.L657:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L617
.L673:
	endbr64
	movq	%rax, %r15
	jmp	.L590
.L672:
	endbr64
	movq	%rax, %r15
	vzeroupper
	jmp	.L592
.L675:
	endbr64
	movq	%rax, %r15
	vzeroupper
	jmp	.L597
.L669:
	endbr64
	movq	%rax, %r15
	vzeroupper
	jmp	.L587
.L678:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L605
.L676:
	endbr64
	movq	%rax, %r15
	jmp	.L595
.L680:
	endbr64
	movq	%rax, %r12
	jmp	.L602
.L681:
	endbr64
	movq	%rax, %r12
	vzeroupper
	jmp	.L542
.L656:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L619
.L655:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L621
.L674:
	endbr64
	movq	%rax, %rbx
	vzeroupper
	jmp	.L594
.L670:
	endbr64
	movq	%rax, %r15
	jmp	.L585
	.section	.gcc_except_table
.LLSDA2312:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSE2312-.LLSDACSB2312
.LLSDACSB2312:
	.uleb128 .LEHB37-.LFB2312
	.uleb128 .LEHE37-.LEHB37
	.uleb128 0
	.uleb128 0
	.uleb128 .LEHB38-.LFB2312
	.uleb128 .LEHE38-.LEHB38
	.uleb128 .L655-.LFB2312
	.uleb128 0
	.uleb128 .LEHB39-.LFB2312
	.uleb128 .LEHE39-.LEHB39
	.uleb128 .L656-.LFB2312
	.uleb128 0
	.uleb128 .LEHB40-.LFB2312
	.uleb128 .LEHE40-.LEHB40
	.uleb128 .L657-.LFB2312
	.uleb128 0
	.uleb128 .LEHB41-.LFB2312
	.uleb128 .LEHE41-.LEHB41
	.uleb128 .L658-.LFB2312
	.uleb128 0
	.uleb128 .LEHB42-.LFB2312
	.uleb128 .LEHE42-.LEHB42
	.uleb128 .L659-.LFB2312
	.uleb128 0
	.uleb128 .LEHB43-.LFB2312
	.uleb128 .LEHE43-.LEHB43
	.uleb128 .L662-.LFB2312
	.uleb128 0
	.uleb128 .LEHB44-.LFB2312
	.uleb128 .LEHE44-.LEHB44
	.uleb128 .L660-.LFB2312
	.uleb128 0
	.uleb128 .LEHB45-.LFB2312
	.uleb128 .LEHE45-.LEHB45
	.uleb128 .L661-.LFB2312
	.uleb128 0
	.uleb128 .LEHB46-.LFB2312
	.uleb128 .LEHE46-.LEHB46
	.uleb128 .L665-.LFB2312
	.uleb128 0
	.uleb128 .LEHB47-.LFB2312
	.uleb128 .LEHE47-.LEHB47
	.uleb128 .L663-.LFB2312
	.uleb128 0
	.uleb128 .LEHB48-.LFB2312
	.uleb128 .LEHE48-.LEHB48
	.uleb128 .L664-.LFB2312
	.uleb128 0
	.uleb128 .LEHB49-.LFB2312
	.uleb128 .LEHE49-.LEHB49
	.uleb128 .L668-.LFB2312
	.uleb128 0
	.uleb128 .LEHB50-.LFB2312
	.uleb128 .LEHE50-.LEHB50
	.uleb128 .L666-.LFB2312
	.uleb128 0
	.uleb128 .LEHB51-.LFB2312
	.uleb128 .LEHE51-.LEHB51
	.uleb128 .L667-.LFB2312
	.uleb128 0
	.uleb128 .LEHB52-.LFB2312
	.uleb128 .LEHE52-.LEHB52
	.uleb128 .L671-.LFB2312
	.uleb128 0
	.uleb128 .LEHB53-.LFB2312
	.uleb128 .LEHE53-.LEHB53
	.uleb128 .L669-.LFB2312
	.uleb128 0
	.uleb128 .LEHB54-.LFB2312
	.uleb128 .LEHE54-.LEHB54
	.uleb128 .L670-.LFB2312
	.uleb128 0
	.uleb128 .LEHB55-.LFB2312
	.uleb128 .LEHE55-.LEHB55
	.uleb128 .L674-.LFB2312
	.uleb128 0
	.uleb128 .LEHB56-.LFB2312
	.uleb128 .LEHE56-.LEHB56
	.uleb128 .L672-.LFB2312
	.uleb128 0
	.uleb128 .LEHB57-.LFB2312
	.uleb128 .LEHE57-.LEHB57
	.uleb128 .L673-.LFB2312
	.uleb128 0
	.uleb128 .LEHB58-.LFB2312
	.uleb128 .LEHE58-.LEHB58
	.uleb128 .L677-.LFB2312
	.uleb128 0
	.uleb128 .LEHB59-.LFB2312
	.uleb128 .LEHE59-.LEHB59
	.uleb128 .L675-.LFB2312
	.uleb128 0
	.uleb128 .LEHB60-.LFB2312
	.uleb128 .LEHE60-.LEHB60
	.uleb128 .L676-.LFB2312
	.uleb128 0
	.uleb128 .LEHB61-.LFB2312
	.uleb128 .LEHE61-.LEHB61
	.uleb128 .L678-.LFB2312
	.uleb128 0
	.uleb128 .LEHB62-.LFB2312
	.uleb128 .LEHE62-.LEHB62
	.uleb128 .L682-.LFB2312
	.uleb128 0
	.uleb128 .LEHB63-.LFB2312
	.uleb128 .LEHE63-.LEHB63
	.uleb128 .L679-.LFB2312
	.uleb128 0
	.uleb128 .LEHB64-.LFB2312
	.uleb128 .LEHE64-.LEHB64
	.uleb128 .L681-.LFB2312
	.uleb128 0
	.uleb128 .LEHB65-.LFB2312
	.uleb128 .LEHE65-.LEHB65
	.uleb128 .L680-.LFB2312
	.uleb128 0
.LLSDACSE2312:
	.text
	.cfi_endproc
	.section	.text.unlikely
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDAC2312
	.type	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13incircleExactENS0_4Vec2ES3_S3_S3_.cold, @function
_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13incircleExactENS0_4Vec2ES3_S3_S3_.cold:
.LFSB2312:
.L600:
	.cfi_escape 0xf,0x3,0x76,0x58,0x6
	.cfi_escape 0x10,0x3,0x2,0x76,0x50
	.cfi_escape 0x10,0x6,0x2,0x76,0
	.cfi_escape 0x10,0xc,0x2,0x76,0x60
	.cfi_escape 0x10,0xd,0x2,0x76,0x68
	.cfi_escape 0x10,0xe,0x2,0x76,0x70
	.cfi_escape 0x10,0xf,0x2,0x76,0x78
	testq	%r12, %r12
	jne	.L889
	vzeroupper
.L601:
	movq	%rbx, %r12
	movq	%r15, %r14
	movq	%r13, %rbx
	jmp	.L542
.L541:
	movq	%r15, %r14
	movq	%r13, %rbx
	vzeroupper
.L542:
	testq	%rbx, %rbx
	je	.L604
	movq	%r14, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L604:
	movq	%r12, %rbx
.L605:
	movq	-176(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L599
	movq	-160(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L599:
	movq	-208(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L594
	movq	-192(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L594:
	movq	-240(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L589
	movq	-224(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L589:
	movq	-272(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L584
	movq	-256(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L584:
	movq	-304(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L579
	movq	-288(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L579:
	movq	-336(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L574
	movq	-320(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L574:
	movq	-368(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L613
	movq	-352(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L613:
	movq	-400(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L615
	movq	-384(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L615:
	movq	-432(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L617
	movq	-416(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L617:
	movq	-464(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L619
	movq	-448(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L619:
	movq	-496(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L621
	movq	-480(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L621:
	movq	-528(%rbp), %rdi
	testq	%rdi, %rdi
	je	.L622
	movq	-512(%rbp), %rsi
	subq	%rdi, %rsi
	call	_ZdlPvm@PLT
.L622:
	movq	-56(%rbp), %rax
	subq	%fs:40, %rax
	jne	.L890
	movq	%rbx, %rdi
.LEHB66:
	call	_Unwind_Resume@PLT
.LEHE66:
.L889:
	movq	-568(%rbp), %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
	jmp	.L601
.L890:
	call	__stack_chk_fail@PLT
.L580:
	testq	%r12, %r12
	je	.L879
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
.L582:
	testq	%rbx, %rbx
	je	.L583
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L583:
	movq	%r15, %rbx
	jmp	.L584
.L879:
	vzeroupper
	jmp	.L582
.L575:
	testq	%r12, %r12
	je	.L878
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
.L577:
	testq	%rbx, %rbx
	je	.L578
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L578:
	movq	%r15, %rbx
	jmp	.L579
.L570:
	testq	%r12, %r12
	je	.L877
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
.L572:
	testq	%rbx, %rbx
	je	.L573
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L573:
	movq	%r15, %rbx
	jmp	.L574
.L877:
	vzeroupper
	jmp	.L572
.L878:
	vzeroupper
	jmp	.L577
.L590:
	testq	%r12, %r12
	je	.L881
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
.L592:
	testq	%rbx, %rbx
	je	.L593
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L593:
	movq	%r15, %rbx
	jmp	.L594
.L881:
	vzeroupper
	jmp	.L592
.L595:
	testq	%r12, %r12
	je	.L882
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
.L597:
	testq	%rbx, %rbx
	je	.L598
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L598:
	movq	%r15, %rbx
	jmp	.L599
.L585:
	testq	%r12, %r12
	je	.L880
	movq	%r14, %rsi
	movq	%r12, %rdi
	subq	%r12, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
.L587:
	testq	%rbx, %rbx
	je	.L588
	movq	%r13, %rsi
	movq	%rbx, %rdi
	subq	%rbx, %rsi
	call	_ZdlPvm@PLT
.L588:
	movq	%r15, %rbx
	jmp	.L589
.L602:
	testq	%r13, %r13
	je	.L884
	movq	%r15, %rsi
	movq	%r13, %rdi
	subq	%r13, %rsi
	vzeroupper
	call	_ZdlPvm@PLT
	jmp	.L542
.L882:
	vzeroupper
	jmp	.L597
.L884:
	vzeroupper
	jmp	.L542
.L880:
	vzeroupper
	jmp	.L587
	.cfi_endproc
.LFE2312:
	.section	.gcc_except_table
.LLSDAC2312:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSEC2312-.LLSDACSBC2312
.LLSDACSBC2312:
	.uleb128 .LEHB66-.LCOLDB18
	.uleb128 .LEHE66-.LEHB66
	.uleb128 0
	.uleb128 0
.LLSDACSEC2312:
	.section	.text.unlikely
	.text
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13incircleExactENS0_4Vec2ES3_S3_S3_, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13incircleExactENS0_4Vec2ES3_S3_S3_
	.section	.text.unlikely
	.size	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13incircleExactENS0_4Vec2ES3_S3_S3_.cold, .-_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13incircleExactENS0_4Vec2ES3_S3_S3_.cold
.LCOLDE18:
	.text
.LHOTE18:
	.p2align 4
	.globl	_ZN8carvetto8geometry6detail12orient2dSignENS0_4Vec2ES2_S2_
	.type	_ZN8carvetto8geometry6detail12orient2dSignENS0_4Vec2ES2_S2_, @function
_ZN8carvetto8geometry6detail12orient2dSignENS0_4Vec2ES2_S2_:
.LFB2313:
	.cfi_startproc
	endbr64
	vsubsd	%xmm5, %xmm3, %xmm6
	vmovapd	%xmm0, %xmm8
	vsubsd	%xmm4, %xmm0, %xmm0
	vsubsd	%xmm4, %xmm2, %xmm7
	vmulsd	%xmm6, %xmm0, %xmm0
	vsubsd	%xmm5, %xmm1, %xmm6
	vmulsd	%xmm7, %xmm6, %xmm6
	vsubsd	%xmm6, %xmm0, %xmm7
	vandpd	.LC2(%rip), %xmm0, %xmm0
	vandpd	.LC2(%rip), %xmm6, %xmm6
	vaddsd	%xmm6, %xmm0, %xmm0
	vmulsd	.LC19(%rip), %xmm0, %xmm0
	vcomisd	%xmm0, %xmm7
	ja	.L893
	vxorpd	.LC12(%rip), %xmm7, %xmm7
	vcomisd	%xmm0, %xmm7
	ja	.L894
	vmovapd	%xmm8, %xmm0
	jmp	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13orient2dExactENS0_4Vec2ES3_S3_
	.p2align 4,,10
	.p2align 3
.L893:
	movl	$1, %eax
	ret
	.p2align 4,,10
	.p2align 3
.L894:
	movl	$-1, %eax
	ret
	.cfi_endproc
.LFE2313:
	.size	_ZN8carvetto8geometry6detail12orient2dSignENS0_4Vec2ES2_S2_, .-_ZN8carvetto8geometry6detail12orient2dSignENS0_4Vec2ES2_S2_
	.p2align 4
	.globl	_ZN8carvetto8geometry6detail12incircleSignENS0_4Vec2ES2_S2_S2_
	.type	_ZN8carvetto8geometry6detail12incircleSignENS0_4Vec2ES2_S2_S2_, @function
_ZN8carvetto8geometry6detail12incircleSignENS0_4Vec2ES2_S2_S2_:
.LFB2314:
	.cfi_startproc
	endbr64
	vsubsd	%xmm7, %xmm3, %xmm15
	vsubsd	%xmm6, %xmm4, %xmm12
	vmovq	%xmm1, %rdi
	vsubsd	%xmm7, %xmm1, %xmm11
	vsubsd	%xmm7, %xmm5, %xmm8
	vmovq	%xmm0, %rax
	vsubsd	%xmm6, %xmm0, %xmm0
	vsubsd	%xmm6, %xmm2, %xmm9
	vmulsd	%xmm12, %xmm15, %xmm1
	vmulsd	%xmm12, %xmm11, %xmm14
	vmulsd	%xmm8, %xmm9, %xmm10
	vmulsd	%xmm11, %xmm11, %xmm13
	vmovq	%xmm1, %rsi
	vmulsd	%xmm8, %xmm0, %xmm1
	vmulsd	%xmm8, %xmm8, %xmm8
	vfmadd231sd	%xmm0, %xmm0, %xmm13
	vmulsd	%xmm15, %xmm0, %xmm0
	vmovq	%xmm1, %rcx
	vmulsd	%xmm15, %xmm15, %xmm1
	vfmadd231sd	%xmm12, %xmm12, %xmm8
	vfmadd231sd	%xmm9, %xmm9, %xmm1
	vmulsd	%xmm9, %xmm11, %xmm9
	vmovq	%xmm1, %rdx
	vmovq	%rsi, %xmm1
	vsubsd	%xmm1, %xmm10, %xmm12
	vmovq	%rcx, %xmm1
	vsubsd	%xmm9, %xmm0, %xmm11
	vsubsd	%xmm1, %xmm14, %xmm15
	vmovq	%rdx, %xmm1
	vandpd	.LC2(%rip), %xmm9, %xmm9
	vandpd	.LC2(%rip), %xmm0, %xmm0
	vandpd	.LC2(%rip), %xmm10, %xmm10
	vmulsd	%xmm1, %xmm15, %xmm15
	vmovq	%rsi, %xmm1
	vaddsd	%xmm9, %xmm0, %xmm0
	vandpd	.LC2(%rip), %xmm1, %xmm9
	vmovq	%rcx, %xmm1
	vaddsd	%xmm9, %xmm10, %xmm10
	vandpd	.LC2(%rip), %xmm14, %xmm9
	vfmadd132sd	%xmm13, %xmm15, %xmm12
	vfmadd132sd	%xmm8, %xmm12, %xmm11
	vandpd	.LC2(%rip), %xmm1, %xmm12
	vmovq	%rdx, %xmm1
	vaddsd	%xmm12, %xmm9, %xmm9
	vmulsd	%xmm1, %xmm9, %xmm9
	vfmadd132sd	%xmm13, %xmm9, %xmm10
	vfmadd132sd	%xmm8, %xmm10, %xmm0
	vmulsd	.LC20(%rip), %xmm0, %xmm0
	vcomisd	%xmm0, %xmm11
	ja	.L897
	vxorpd	.LC12(%rip), %xmm11, %xmm11
	vcomisd	%xmm0, %xmm11
	ja	.L898
	vmovq	%rax, %xmm0
	vmovq	%rdi, %xmm1
	jmp	_ZN8carvetto8geometry6detail12_GLOBAL__N_1L13incircleExactENS0_4Vec2ES3_S3_S3_
	.p2align 4,,10
	.p2align 3
.L897:
	movl	$1, %eax
	ret
	.p2align 4,,10
	.p2align 3
.L898:
	movl	$-1, %eax
	ret
	.cfi_endproc
.LFE2314:
	.size	_ZN8carvetto8geometry6detail12incircleSignENS0_4Vec2ES2_S2_S2_, .-_ZN8carvetto8geometry6detail12incircleSignENS0_4Vec2ES2_S2_S2_
	.section	.rodata.cst16,"aM",@progbits,16
	.align 16
.LC2:
	.long	-1
	.long	2147483647
	.long	0
	.long	0
	.section	.rodata.cst8,"aM",@progbits,8
	.align 8
.LC7:
	.long	33554432
	.long	1101004800
	.set	.LC10,.LC12
	.section	.rodata.cst16
	.align 16
.LC12:
	.long	0
	.long	-2147483648
	.long	0
	.long	0
	.section	.rodata.cst32,"aM",@progbits,32
	.align 32
.LC14:
	.quad	0
	.quad	-1
	.quad	-2
	.quad	-3
	.section	.rodata.cst8
	.align 8
.LC15:
	.quad	-4
	.align 8
.LC19:
	.long	0
	.long	1019215872
	.align 8
.LC20:
	.long	0
	.long	1020788736
	.hidden	DW.ref.__gxx_personality_v0
	.weak	DW.ref.__gxx_personality_v0
	.section	.data.rel.local.DW.ref.__gxx_personality_v0,"awG",@progbits,DW.ref.__gxx_personality_v0,comdat
	.align 8
	.type	DW.ref.__gxx_personality_v0, @object
	.size	DW.ref.__gxx_personality_v0, 8
DW.ref.__gxx_personality_v0:
	.quad	__gxx_personality_v0
	.globl	__gxx_personality_v0
	.ident	"GCC: (Ubuntu 15.2.0-16ubuntu1) 15.2.0"
	.section	.note.GNU-stack,"",@progbits
	.section	.note.gnu.property,"a"
	.align 8
	.long	1f - 0f
	.long	4f - 1f
	.long	5
0:
	.string	"GNU"
1:
	.align 8
	.long	0xc0000002
	.long	3f - 2f
2:
	.long	0x3
3:
	.align 8
4:
