#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include "v68.h"
#include "m68k.h"
#include "ide.h"


static uint8_t ram[512 * 1024];

struct ide_controller *ide;

uint8_t timer_v = 1;

#define IOBASE		0x00F00000
#define IOIDE_START	0x00F01000
#define IOIDE_END	0x00F0100F

#define MMU_MASK	0x00F02000
#define MMU_ROOT	0x00F02010
#define MMU_FAULT	0x00F02020

#define SERIO_IN	0x00F03000
#define SERIO_OUT	0x00F03000
#define SERIO_STATUS	0x00F03010

#define TIMER_IO	0x00F04000

#define IRQ_MMU 	7
#define IRQ_TIMER	6

/* Read/write macros */
#define READ_BYTE(BASE, ADDR) (BASE)[ADDR]
#define READ_WORD(BASE, ADDR) (((BASE)[ADDR]<<8) | \
			(BASE)[(ADDR)+1])
#define READ_LONG(BASE, ADDR) (((BASE)[ADDR]<<24) | \
			((BASE)[(ADDR)+1]<<16) | \
			((BASE)[(ADDR)+2]<<8) | \
			(BASE)[(ADDR)+3])

#define WRITE_BYTE(BASE, ADDR, VAL) (BASE)[ADDR] = (VAL)&0xff
#define WRITE_WORD(BASE, ADDR, VAL) (BASE)[ADDR] = ((VAL)>>8) & 0xff; \
			(BASE)[(ADDR)+1] = (VAL)&0xff
#define WRITE_LONG(BASE, ADDR, VAL) (BASE)[ADDR] = ((VAL)>>24) & 0xff; \
			(BASE)[(ADDR)+1] = ((VAL)>>16)&0xff; \
			(BASE)[(ADDR)+2] = ((VAL)>>8)&0xff; \
			(BASE)[(ADDR)+3] = (VAL)&0xff 

uint8_t fc;

/* Simple polled serial port */

static unsigned int check_chario(void)
{
	fd_set i, o;
	struct timeval tv;
	unsigned int r = 0;
	
	FD_ZERO(&i);
	FD_SET(0, &i);
	FD_ZERO(&o);
	FD_SET(1, &o);
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	if (select(2, &i, NULL, NULL, &tv) == -1) {
		perror("select");
		exit(1);
	}
	if (FD_ISSET(0, &i))
		r |= 1;
	if (FD_ISSET(1, &o))
		r |= 2;
	return r;
}

static unsigned int next_char(void)
{
	char c;
	if (read(0, &c, 1) != 1)
		return 0xFF;
	return c;
}

static void char_out(unsigned int c)
{
	putchar(c);
	fflush(stdout);
}

/* Interrupt controller */

static unsigned int irq_pending;

static void irq_compute(void)
{
	int i;
	if (irq_pending) {
		for (i = 7; i >= 0; i--) {
			if (irq_pending & (1 << i)) {
				m68k_set_irq(i);
				return;
			}
		}
	} else
		m68k_set_irq(0);
}

int cpu_irq_ack(int level)
{
	if (!(irq_pending & (1 << level)))
		return M68K_INT_ACK_SPURIOUS;
	if (level == IRQ_MMU)
		return M68K_INT_ACK_AUTOVECTOR;
	if (level == IRQ_TIMER)
		return M68K_INT_ACK_AUTOVECTOR;
	return M68K_INT_ACK_SPURIOUS;
}

/* Minimal MMU */

static uint32_t mmu_mask, mmu_root, mmu_fault;

static unsigned int translate(unsigned int addr)
{
	unsigned int pa = (addr & mmu_mask) ^ mmu_root;
	if ((addr & ~mmu_mask) && (addr & ~mmu_mask) != ~mmu_mask) {
		/* bus_error */
		if (mmu_fault == 0) {
			mmu_fault = 1;
			irq_pending |= 1 << IRQ_MMU;
			irq_compute();
		}
		return 0xFFFFFFFF;
	}
	return pa;
}

/* CPU bus decodes */

static unsigned int do_io_readb(unsigned int address)
{
	unsigned int r;

	if (address >= IOIDE_START && address <= IOIDE_END)
		return ide_read8(ide, (address - IOIDE_START) / 2);

	switch(address) {
	case MMU_MASK:
		return mmu_mask >> 20;
	case MMU_MASK + 1:
		return mmu_mask >> 12;
	case MMU_ROOT:
		return mmu_root >> 20;
	case MMU_ROOT+1:
		return mmu_root >> 12;
	case MMU_FAULT:
		r = mmu_fault;
		mmu_fault = 0;
		return r;
	case SERIO_IN:
		return next_char();
	case SERIO_STATUS:
		return check_chario();
	case TIMER_IO:
		irq_pending &= ~ (1 << IRQ_TIMER);
		irq_compute();
		return 0x00;
	}
	/* bus error ? */
	return 0xFF;
}

static void do_io_writeb(unsigned int address, unsigned int value)
{
//	printf("IOwriteB %x = %d\n", address, value);
	if (address >= IOIDE_START && address <= IOIDE_END)
		ide_write8(ide, (address - IOIDE_START) / 2, value);
	else switch(address) {
	case MMU_MASK:
		mmu_mask &= (0xFF << 20);
		mmu_mask |= (value << 20);
		break;
	case MMU_MASK + 1:
		mmu_mask &= (0xFF << 12);
		mmu_mask |= (value << 12);
		break;
	case MMU_ROOT:
		mmu_root &= (0xFF << 20);
		mmu_root |= (value << 20);
		break;
	case MMU_ROOT+1:
		mmu_root &= (0xFF << 12);
		mmu_root |= (value << 12);
		break;
	case MMU_FAULT:
		break;
	case SERIO_OUT:
		char_out(value);
		break;
	case TIMER_IO:
		timer_v = value;
		break;
	default: ;
		/* bus error ? */
	}
}

/* Read data from RAM, ROM, or a device */
unsigned int cpu_read_byte(unsigned int address)
{
	if (!(fc & 4))
		address = translate(address);
	if (address < sizeof(ram))
		return ram[address];
	if (address >= IOBASE)
		return do_io_readb(address);
	/* Bus error ?? */
//	printf("BUS %d\n", address);
	return 0xFF;
}

unsigned int cpu_read_word(unsigned int address)
{
	unsigned int vaddress = address;
	/* We can do this as one because we know the address is even
	   aligned so the two bytes translate adjacent */
	if (!(fc & 4))
		vaddress = translate(vaddress);
	if (vaddress < sizeof(ram) - 1)
		return READ_WORD(ram, vaddress);
	if (address >= IOIDE_START && address <= IOIDE_END)
		return ide_read16(ide, (address - IOIDE_START) / 2);
	/* Corner cases */
	return (cpu_read_byte(address) << 8) | cpu_read_byte(address + 1);
}

unsigned int cpu_read_word_dasm(unsigned int address)
{	
	return cpu_read_word(address);
}

unsigned int cpu_read_long(unsigned int address)
{
//	printf("RL %x\n", address);
	return (cpu_read_word(address) << 16) | cpu_read_word(address + 2);
}

unsigned int cpu_read_long_dasm(unsigned int address)
{	
	return cpu_read_long(address);
}

void cpu_write_byte(unsigned int address, unsigned int value)
{
	if (!(fc & 4)) {
		address = translate(address);
		if (address < 0x1000) {
			printf("WFAULT %x %x\n", address, value & 0xFF);
			return;
		}
	}
	if (address < sizeof(ram))
		ram[address] = value;
	else if (address >= IOBASE)
		do_io_writeb(address, (value & 0xFF));
	/* else Bus error ? */
}

void cpu_write_word(unsigned int address, unsigned int value)
{
	unsigned int vaddress = address;
	/* We can do this as one because we know the address is even
	   aligned so the two bytes translate adjacent */
	if (!(fc & 4)) {
		vaddress = translate(vaddress);
		if (vaddress < 0x1000) {
			printf("WFAULT %x %x\n", vaddress, value & 0xFFFF);
			return;
		}
	}
	if (vaddress < sizeof(ram) - 1) {
		WRITE_WORD(ram, vaddress, value);
	} else 	if (address >= IOIDE_START && address <= IOIDE_END)
		ide_write16(ide, (address - IOIDE_START) / 2, value);
	else {
		/* Corner cases */
		cpu_write_byte(address, value >> 8);
		cpu_write_byte(address + 1, value & 0xFF);
	}
}

void cpu_write_long(unsigned int address, unsigned int value)
{
	cpu_write_word(address, value >> 16);
	cpu_write_word(address + 2, value & 0xFFFF);
}

void cpu_write_pd(unsigned int address, unsigned int value)
{
	cpu_write_word(address + 2, value & 0xFFFF);
	cpu_write_word(address, value >> 16);
}


void cpu_instr_callback(void)
{
}

static struct timespec last_time;

static void device_init(void)
{
	clock_gettime(CLOCK_MONOTONIC, &last_time);
	irq_pending = 0;
	mmu_mask = 0xFFFFFFFF;
	mmu_root = 0;
	mmu_fault = 0;
	ide_reset_begin(ide);
}

static void device_update(void)
{
	struct timespec tv, tmp;
	unsigned long n;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	tmp.tv_sec = tv.tv_sec - last_time.tv_sec;
	tmp.tv_nsec = tv.tv_nsec - last_time.tv_nsec;
	/* Difference in hundredths */
	n = tmp.tv_sec * 100 + tmp.tv_nsec/10000000;
	if (timer_v && n >= timer_v) {
		last_time = tv;
		irq_pending |= 1 << IRQ_TIMER;
		irq_compute();
	}
}

void cpu_pulse_reset(void)
{
	device_init();
}

int main(int argc, char* argv[])
{
	int fd;

	if(argc > 2) {
		printf("Usage: v68 <program file>\n");
		exit(-1);
	}

	if (argc == 2) {
		if((fd = open(argv[1], O_RDONLY)) == -1) {
			perror(argv[1]);
			exit(1);
		}

		if(read(fd, ram, sizeof(ram)) < 0) {
			perror("read");
			exit(1);
		}
		close(fd);
	} else {
		/* Boot data into memory */
		fd = open("boot.dat", O_RDONLY);
		if (fd == -1) {
			perror("boot.dat");
			exit(1);
		}
		if (read(fd, ram, 0x1000) != 0x1000) {
			perror("boot.dat");
			exit(1);
		}
		close(fd);
	}
	
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);

	fd = open("disk.img", O_RDONLY);
	if (fd == -1) {
		perror("disk.img");
		exit(1);
	}
	ide = ide_allocate("hd0");
	if (ide == NULL)
		exit(1);
	if (ide_attach(ide, 0, fd))
		exit(1);

	/* Init devices */
	device_init();

	m68k_pulse_reset();

	while(1) {
		m68k_execute(100000);
		device_update();
	}
}

