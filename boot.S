/*
 *	"BOOT" ROM
 */

		.globl _start

		.text

.mri	1

IDEBASE		equ	$00F01000
MMUMASK		equ	$00F02000
MMUROOT		equ	$00F02010
MMUFAULT	equ	$00F02020
SERIN		equ	$00F03000
SEROUT		equ	$00F03000
SERSTAT		equ	$00F03010

MMUIRQ		equ	7
TIMERIRQ	equ	6


; Vectors at address 0

		; Boot vectors (0,1)
		long	$00008000	; Boot SP
		long	boot		; Boot PC

		; System exceptions (2-15)
		long	bomb		; Bus error
		long	bomb		; Address error
		long	bomb		; Illegal
		long	bomb		; Div 0 (int)
		long	bomb		; CHK/CHK2
		long	bomb		; TRAPcc/TRAPV
		long	bomb		; Privilege violation
		long	bomb		; Trace trap
		long	bomb		; A line
		long	bomb		; F line
		long	bomb		; unused
		long	bomb		; unused (no coprocessor on a 68000)
		long	bomb		; unused (format on 68010+)
		long	bomb		; uninitialized interrupt vector

		; Reserved (16-23)
		long	bomb		; spare 16 
		long	bomb		; spare 17
		long	bomb		; spare 18
		long	bomb		; spare 19
		long	bomb		; spare 20
		long	bomb		; spare 21
		long	bomb		; spare 22
		long	bomb		; spare 23

		; Spurious IRQ (24)
		long	bomb		; spare 24

		; Autovector IRQ (25-31)
		long	ignore		; Level 1
		long	ignore		; Level 2
		long	ignore		; Level 3
		long	ignore		; Level 4
		long	ignore		; Level 5
		long	ignore		; Level 6
		long	ignore		; Level 7

		; Traps (32-47)
		long	bomb		; Trap 0
		long 	bomb		; Trap 1
		long	bomb		; Trap 2
		long	bomb		; Trap 3
		long	bomb		; Trap 4
		long	bomb		; Trap 5
		long	bomb		; Trap 6
		long	bomb		; Trap 7
		long	bomb		; Trap 8
		long 	bomb		; Trap 9
		long	bomb		; Trap 10
		long	bomb		; Trap 11
		long	bomb		; Trap 12
		long	bomb		; Trap 13
		long	bomb		; Trap 14
		long	bomb		; Trap 15

		; FPU 48-55(unused)
		long	bomb		; BSUC
		long	bomb		; Inexact
		long	bomb		; Div0
		long	bomb		; Underflow
		long	bomb		; Operand error
		long	bomb		; Overflow
		long	bomb		; NaN
		long	bomb		; Unimplemented data (68040+)

		; MMU 56-58 (unused - not on 68000)
		long	bomb		; Configuration error
		long	bomb		; Illegal operation
		long	bomb		; Access level violation

		; Reserved (59-63)
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb

		; User vectors 64-255
		long	bomb		; 64
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 72
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 80
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 88
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 96
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 104
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 112
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 120
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 128
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 136
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 144
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 152
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 160
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 168
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 176
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 184
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 192
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 200
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 208
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 216
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 224
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 232
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 240
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb		; 248
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb
		long	bomb

		; End of vector table (Address $0400)
_start:
boot:
		; Print a first letter so the world knows we got this far
		move.l	#SEROUT,a0
		move.b	#'V',(a0)
		move.l	#banner,a3
		bsr	strout
		; Size memory
		move.l	#$1000,a1
		moveq	#0,d1
sizeit:		clr.l	(a1)
		tst.l	(a1)+		; it will go FFFFFFFF when we go off
		beq	sizeit		; the end

command:
		move.l	#prompt,a3
		bsr	strout
		move.l	#$7000,a2
		moveq	#80,d2
		bsr	strin
		move.b	$7000,d0
		beq	command
		cmp.b	#'b',d0
		beq	cmd_boot
		cmp.b	#'r',d0
		beq	cmd_reset
		move.l	#badcmd,a3
		bsr	strout
		bra	command

cmd_boot:
		;
		; Now try and load the boot block from IDE
		;

		move.l	#ideload,a3
		bsr	strout

		move.l	#$1000,a0	; Store data at $1000
		clr.l	d0		; Disk 0
		clr.l	d1		; Block 0

		bsr	diskload

		tst.w	d0
		beq	boottime
		move.l	#diskfault,a3
		bsr	strout


boottime:
		; And run it - there is much to flesh out in the boot ROM
		; yet !
		move.l	$1000,d0
		cmp.l	#$15C0DE68,d0
		beq	bootit
		move.l	#notboot,a3
		bsr	strout
		bra	halt
bootit:
		move.l	#gogogo,a3
		bsr	strout
		move.l	#bootservices,a6
		moveq	#0,d0		; D0 = disk we booted from
		jmp	$1004		; Entered with A6 as service vector

		; Trap or fault taken
bomb:
		move.l #unexpected,a3
		move.l #SEROUT,a0
		bsr strout
halt:		move.l #$8000,sp
		move.l #hitkey,a3
		bsr strout
		bsr chrin
cmd_reset:
		move.l #$8000,sp
		reset
		bra boot

ignore:		rte

;
;		Library Routines
;		Trash input registers and A0/D0/A1/D1
;

;
;		Print zero terminated string in A3
;
strout:
		move.l	#SEROUT,a0
		move.b	(a3)+,d0
		tst.b	d0
		beq	strend
strw1:		btst	#1,10(a0)
		beq strw1
		move.b	d0,(a0)
		bra	strout
strend:		rts

;
;		Print character in D0
;
chrout:
		move.l	#SEROUT,a0
chrw1:		btst	#1,10(a0)
		beq chrw1
		move.b	d0,(a0)
		rts

;
;		Read a character into D0
;		
chrin:
		move.l	#SERIN,a0
chrr1:		btst	#0,10(a0)
		beq chrr1
		move.b	(a0),d0
		and.b	#$7F,d0
		rts

;
;	Check input status
;
;		Bit 0 = read ready bit 1 = write ready
;
chrpoll:	move.b	SERIN,d0
		and.b	#3,d0
		rts
;
;	Read a line of input
;
;	A2 = buffer, D2.w = length of buffer
;
strin:		move.l	a3,-(sp)
		move.l	a2,a3
		subq	#1,d2		; reserve space for CR
strinl:		bsr	chrin
		cmp.b	#8,d0
		beq	strdel
		cmp.b	#127,d0
		beq	strdel
		cmp.b	#10,d0
		beq	strent
		cmp.b	#32,d0
		blt	strinl
		tst.w	d2
		beq	strinl
		move.b	d0,(a2)+
		bsr	chrout
		subq	#1,d2
		bra	strinl
strdel:		cmpa	a2,a3
		beq	strinl
		move.b 	#8,d0
		bsr	chrout
		move.b	#' ',d0
		bsr	chrout
		move.b	#8,d0
		bsr	chrout
		addq	#1,d2
		sub.l	#1,a2
		bra	strinl
strent:		move.b	#13,d0
		bsr	chrout
		move.b	#10,d0
		bsr	chrout
		clr.b	(a2)
		move.l	(sp)+,a3
		rts
;
;	Load block D1 from disk D0 (currently disk must be 0/1) into A0
;
;	FIXME: needs timeouts
;
;	Returns D0 = 0 on success, non zero on error
;
diskload:
		move.l	#IDEBASE,a1
diskwait1:		
		btst #7,15(a1)		; Wait for drive to leave reset
		bne diskwait1

		move.b	d1,7(a1)	; disk block n (assumes LBA)
		ror.l	#8,d1
		move.b	d1,9(a1)
		ror.l	#8,d1
		move.b	d1,11(a1)
		ror.l	#8,d1
		and.b	#$0F,d1
		tst.w	d0
		beq disk0sel
		cmp.w	#1,d0
		bne	diskerr		; No such drive
		or.b	#$10,d1		; Disk select bit
disk0sel:
		or.b	#$E0,d1
		move.b	d1,13(a1)	; drive 0 E0 / drive 1 F0
diskwait2:				; Wait for ready
		btst #0,15(a1)		; Error bit ?
		bne 	diskerr
		btst #6,15(a1)
		beq diskwait2

		move.b	#1,5(a1)	; for one sector only
		move.b	#$20,15(a1)	; read sector

diskwait3:				; Wait for DRQ
		btst #0,15(a1)		; Error bit ?
		bne 	diskerr
		btst #3,15(a1)		; DRQ
		beq diskwait3

		move.w	#255,d0
diskblk:	move.w	(a1),d1		; Copy the bytes
		move.w	d1,(a0)+
		dbeq	d0,diskblk
		moveq	#0,d0
		rts
diskerr:	moveq	#1,d0
		rts



;
;	Expose our minimal services to the code we boot
;
bootservices:	jmp	halt
		jmp	chrout
		jmp	strout
		jmp	chrin
		jmp	chrpoll
		jmp	strin
		jmp	diskload
		

banner:		asciz "68 Boot Services\r\nRevision 0.1 Alpha\r\n\r\nClearing memory\r\n"
ideload:	asciz "Booting from IDE disk 0\r\n"
gogogo:		asciz "OK\r\n"
notboot:	asciz "Not a bootable device\r\n"
unexpected:	asciz "Unexpected trap: halting\r\n"
diskfault:	asciz "Disk error\r\n"
hitkey:		asciz "Hit a key to restart\r\n"
prompt:		asciz "> "
badcmd:		asciz "Unknown command\r\n"
