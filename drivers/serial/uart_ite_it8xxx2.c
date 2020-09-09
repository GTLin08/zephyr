/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <kernel.h>
#include <arch/cpu.h>
#include <zephyr/types.h>
#include <soc.h>
#include <init.h>
#include <toolchain.h>
#include <linker/sections.h>
#include <drivers/uart.h>
#include <sys/sys_io.h>

/**
 * struct declaration
 */
#define T_UART_1	0
#define T_UART_2	1
#define T_UART_MAX_IDX	2

#define STATUS_SUCCESS		0
#define STATUS_FAIL		(-1)
#define STATUS_IRQ_READY	1
#define STATUS_WARN_BAUDRATE	0x1001

enum _T_UART_BAUDRATE_INDEX_ {
	t_br_4800   = 0,
	t_br_9600,
	t_br_14400,
	t_br_19200,
	t_br_38400,
	t_br_57600,
	t_br_115200,
	t_br_230400,
	t_br_460800,

	t_br_max_idx,
};

struct T_UART_BAUDRATE_TABLE {
	int baud_rate;
	enum _T_UART_BAUDRATE_INDEX_ index;
};

struct T_UART_BAUDRATE_TABLE	baudrate_table[t_br_max_idx] = {
	{4800, t_br_4800},
	{9600, t_br_9600},
	{14400, t_br_14400},
	{19200, t_br_19200},
	{38400, t_br_38400},
	{57600, t_br_57600},
	{115200, t_br_115200},
	{230400, t_br_230400},
	{460800, t_br_460800}
};

/**
 * Structure define
 */
struct T_ITEUART {
	unsigned char rbrthrdlb;	/* 0x00 */
	unsigned char ierdmb;		/* 0x01 */
	unsigned char iirfcrafr;	/* 0x02 */
	unsigned char ULCR;		/* 0x03 */
	unsigned char UMCR;		/* 0x04 */
	unsigned char ULSR;		/* 0x05 */
	unsigned char UMSR;		/* 0x06 */
	unsigned char USCR;		/* 0x07 */
};

#define URBR	rbrthrdlb
#define UTHR	rbrthrdlb
#define UDLL	rbrthrdlb
#define UIER	ierdmb
#define UDLM	ierdmb
#define UIIR	iirfcrafr
#define UFCR	iirfcrafr
#define UAFR	iirfcrafr

/**
 * uart structure
 */
struct T_UartStruct {
	volatile struct T_ITEUART	*mUartBase;
	volatile uint8_t		*rx;
	volatile uint8_t		*tx;
	volatile uint8_t		*rts;
	volatile uint8_t		*cts;
	uint8_t				ucGCR1;
	uint8_t				ucGCR6;
	uint8_t				ucGCR8;
};

struct T_BaudrateStruct {
	uint8_t			baud_int;
	uint8_t			baud_den;
	uint8_t			baud_num;
	uint8_t			baud_udll;
	uint8_t			baud_udlm;
};

/**
 *
 * Macro define
 *
 */
/*  UIER - 0x01 */
#define IER_EMSI		BIT(3)	/* Modem Status Interrupt */
#define IER_ERLSI		BIT(2)	/* Recv Line Status Interrupt */
#define IER_ETHREI		BIT(1)	/* Transmitter Holding Interrupt */
#define IER_ERDVI		BIT(0)	/* Recv Data Available Interrupt */

/*  UFCR - 0x02, WO */
#define FCR_FIFO_14		0xC0	/* 14 / 56 Byte FIFO Trigger*/
#define FCR_FIFO_8		0x80	/*  8 / 32 Byte FIFO Trigger*/
#define FCR_FIFO_4		0x40	/*  4 / 16 Byte FIFO Trigger*/
#define FCR_FIFO_1		0x00	/*  1 /  1 Byte FIFO Trigger*/
#define FCR_64B_FIFO_EN		BIT(5)	/* 64 Bytes FIFO Enable */
#define FCR_XFRST		BIT(2)	/* transmitter soft reset */
#define FCR_RFRST		BIT(1)	/* receiver soft reset */
#define FCR_FIFO_EN		BIT(0)	/* fifo enable */

/*  UIIR - 0x02, RO */
#define IIR_TIME_OUT_INDI	0x0C	/* character timer-out */
#define IIR_RECV_LINE_S		0x06	/* received line status */
#define IIR_RECV_AVAILABLE	0x04	/* received data available */
#define IIR_TX_HOLD_EMPTY	0x02	/* transmitter holding register */
#define IIR_NONE		0x01	/* none */
#define IIR_MODEM_STATUS	0x00	/* modem status */
#define IIR_MASK		0x0F	/* none */
#define IIR_MASK_ID		0x06	/* interrupt ID mask */

/*  ULCR - 0x03 */
#define LCR_DLAB		BIT(7)		/* Divisor Latch Bit */
#define LCR_BREAK		BIT(6)		/* Break Control */
#define LCR_SP			BIT(5)		/* Stick Parity Bit */
#define LCR_EPS			BIT(4)		/* Even Parity Select */
#define LCR_PEN			BIT(3)		/* Parity eneble */
#define LCR_STB			BIT(2)		/* Number of stop Bits */
#define LCR_WLS_MSK		(BIT(0) | BIT(1))	/* char length mask */
#define LCR_WLS_5		0x00		/* 5 bit character length */
#define LCR_WLS_6		0x01		/* 6 bit character length */
#define LCR_WLS_7		0x02		/* 7 bit character length */
#define LCR_WLS_8		0x03		/* 8 bit character length */
#define LCR_8N1			(LCR_WLS_8)

/*  UMCR - 0x04 */
#define MCR_AFE			BIT(5)	/* Auto flow control enable */
#define MCR_LOOP		BIT(4)	/* Local loopback mode enable */
#define MCR_OUT2		BIT(3)	/* output2 pin */
#define MCR_OUT1		BIT(2)	/* output1 pin */
#define MCR_RTS			BIT(1)	/* Readyu to Send */
#define MCR_DTR			BIT(0)

/*  ULSR - 0x05 */
#define LSR_ERR			BIT(7)	/* Error */
#define LSR_TEMT		BIT(6)	/* Xmitter empty */
#define LSR_THRE		BIT(5)	/* Xmit holding register empty */
#define LSR_BI			BIT(4)	/* Break Interrupt*/
#define LSR_FE			BIT(3)	/* Framing error */
#define LSR_PE			BIT(2)	/* Parity error */
#define LSR_OE			BIT(1)	/* Overrun Error */
#define LSR_DR			BIT(0)	/* Data ready */

/* constants for modem status register */
/* MSR - 0x06 */
#define MSR_DCTS		0x01	/* cts change */
#define MSR_DDSR		0x02	/* dsr change */
#define MSR_DRI			0x04	/* ring change */
#define MSR_DDCD		0x08	/* data carrier change */
#define MSR_CTS			0x10	/* complement of cts */
#define MSR_DSR			0x20	/* complement of dsr */
#define MSR_RI			0x40	/* complement of ring signal */
#define MSR_DCD			0x80	/* complement of dcd */

/*  UDMACR - 0x10 */
#define DMACR_RX_BUF_FULL_INT_EN	BIT(7)
#define DMACR_TX_BUF_FULL_INT_EN	BIT(6)
#define DMACR_TX_FINISH_INT_EN		BIT(5)
#define DMACR_RX_FINISH_INT_EN		BIT(4)
#define DMACR_TX_DMA_MODE_EN		BIT(1)
#define DMACR_RX_DMA_MODE_EN		BIT(0)

/*  UDMAIR - 0x11 */
#define DMACR_RX_BUF_1_FULL_INT_FG	BIT(7)
#define DMACR_RX_BUF_0_FULL_INT_FG	BIT(6)
#define DMACR_TX_DMA_FINISH_FG		BIT(1)	/* R/W0 */
#define DMACR_RX_DMA_FINISH_FG		BIT(0)	/* R/W0 */
#define DEF_UART2_ENABLE_CTS_RTS	TRUE	/*turn on uart 2 flow control*/
#define DEF_UART1_ENABLE_CTS_RTS	FALSE	/*turn off uart 1 flow control*/
#define IIRC(dev)			(DEV_DATA(dev)->iir_cache)
#define GPH1VS				BIT(1)
#define GPH2VS				BIT(0)

/**
 * Const Variable
 *
 * GCR1 : uart enable
 * GCR6 : each pin enable, BIT0:SIN, BIT1:SOUT
 * GCR8 : total pin number
 */
static const struct T_UartStruct mComPortTb[T_UART_MAX_IDX] = {
	/* UART1 */
#if DEF_UART1_ENABLE_CTS_RTS
	{ ((volatile struct T_ITEUART *)REG_UART1_BASE), &GPCRB0,
	  &GPCRB1, &GPCRF3, &GPCRD5, (0x10 << 0), (BIT(0) | BIT(1)), 0x03 },
#else
	{ ((volatile struct T_ITEUART *)REG_UART1_BASE), &GPCRB0,
	  &GPCRB1, &GPCRF3, &GPCRD5, (0x01 << 0), (BIT(1)), 0x03 },
#endif

	/* UART2 */
	{ ((volatile struct T_ITEUART *)REG_UART2_BASE), &GPCRH1,
	  &GPCRH2, &GPCRE5, &GPCRI7, (0x10 << 2), (BIT(2) | BIT(3)), 0x0C },
};

static const struct T_BaudrateStruct mBaudrateTb[t_br_max_idx] = {
/**                       1,500,000
 *              NUM      (3,000,000)         1
 * ( (INT+1) + ----- ) = ----------- x --------------
 *              DEN       baud rate    ((DLM<<8)|DLL)
 *
 *    INT, DEN, NUM,  DLL,  DLM
 */
	{ 155, 100,  25,   0x18, 0x00 },    /* 0x00 - 4800 */
	{ 155, 100,  25,   0x10, 0x00 },    /* 0x01 - 9600 */
	{ 103,   6,   1,   0x0c, 0x00 },    /* 0x02 - 14400 */
	{  77,   8,   1,   0x06, 0x00 },    /* 0x03 - 19200 */
	{  38,  16,   1,   0x03, 0x00 },    /* 0x04 - 38400 */
	{  25,  24,   1,   0x02, 0x00 },    /* 0x05 - 57600 */
	{  12,  48,   1,   0x01, 0x00 },    /* 0x06 - 115200 */
	{   5,  96,  49,   0x02, 0x80 },    /* 0x07 - 230400 */
	{   2, 192,  49,   0x01, 0x80 },    /* 0x08 - 460800 */
};

/**
 *
 * uart_putc
 * put char out
 * @return
 *	STATUS_SUCCESS
 *	STATUS_FAIL
 * @parameter
 *	chan - uart channel
 *	data - uart put out data
 */
static uint8_t uart_putc(uint8_t chan, uint8_t data)
{
	unsigned long u32temp = 0;

	while ((mComPortTb[chan].mUartBase->ULSR & LSR_THRE) == 0) {
		if (u32temp == 0x100) {
			return STATUS_FAIL;
		}
		u32temp++;
	}
	mComPortTb[chan].mUartBase->UTHR = data;
	return STATUS_SUCCESS;
}

/**
 *
 * uart_getc
 * get char from outer
 * @return
 *	STATUS_SUCCESS
 *	STATUS_FAIL
 * @parameter
 *	chan - uart channel
 *	data - uart get in data
 */
static uint8_t uart_getc(uint8_t chan, uint8_t *data)
{
	unsigned long u32temp = 0;

	while ((mComPortTb[chan].mUartBase->ULSR & LSR_DR) == 0) {
		if (u32temp == 0x100) {
			return STATUS_FAIL;
		}
		u32temp++;
	}

	*data = mComPortTb[chan].mUartBase->UTHR;
	return STATUS_SUCCESS;
}

/**
 *
 * uart_is_xmit_empty
 * get status to check data has been sent
 * @return
 *	TRUE - All data has been sent
 *	FALSE
 * @parameter
 *	chan - uart channel
 *
 */
static uint8_t uart_is_xmit_empty(uint8_t chan)
{
	return IS_MASK_SET(mComPortTb[chan].mUartBase->ULSR, LSR_TEMT);
}

/**
 *
 * uart_init
 * initial uart: setting register and baud rate
 * @return
 *	N/A
 * @parameter
 *	chan - uart channel
 *	baudrate - uart baud rate
 *	is_enable_parity - enable parity
 *	is_event_parity - event parity
 *	is_onestop_bit - one stop bit
 */
static void uart_init(uint8_t chan, uint8_t baudrate,
		      uint8_t is_enable_parity, uint8_t is_event_parity,
		      uint8_t is_onestop_bit)
{
	volatile struct T_ITEUART  *pComPort
		= mComPortTb[chan].mUartBase;
	const struct T_BaudrateStruct *pBR
		= &mBaudrateTb[baudrate];
	unsigned char ucTemp;

	/* Enable auto gating
	 * UART Clock Gating (UART12CG) 0: Operation,
	 * 1: Clocks to UART1/UART2 modules are gated.
	 */
	CGCTRL3R &= ~(BIT(2));
	AUTOCG   &= ~(BIT(6 - chan));

	/* Enable uart module */
	RSTDMMC  |= BIT(3 - chan);      /* Set EC side control */
	RSTC4    = BIT(1 + chan);       /* W-One to reset controller */
	GCR1    |= mComPortTb[chan].ucGCR1;
	GCR6    |= mComPortTb[chan].ucGCR6;
	GCR8    |= mComPortTb[chan].ucGCR8;
	if (chan == T_UART_2) {
		GCR21 &= ~(GPH1VS | GPH2VS);
		SET_MASK(UART2_ECSPMR, BIT(1));
		if (DEF_UART2_ENABLE_CTS_RTS) {
			if (mComPortTb[chan].cts) {
				*mComPortTb[chan].cts = 0x00;
			}
			if (mComPortTb[chan].rts) {
				*mComPortTb[chan].rts = 0x00;
				SET_MASK(pComPort->UMCR, MCR_AFE);
				ucTemp = UART2_MSR;	/* Read Clear */
			}
		}
	} else if ((chan == T_UART_1) && DEF_UART1_ENABLE_CTS_RTS) {
		if (mComPortTb[chan].cts) {
			*mComPortTb[chan].cts = 0x00;
		}
		if (mComPortTb[chan].rts) {
			*mComPortTb[chan].rts = 0x00;
			SET_MASK(pComPort->UMCR, MCR_AFE);
			ucTemp = pComPort->UMSR;	/* Read Clear */
		}
	}
	if (mComPortTb[chan].rx) {
		*mComPortTb[chan].rx = 0x00;
	}
	if (mComPortTb[chan].tx) {
		*mComPortTb[chan].tx = 0x00;
	}

	/* set baudrate */
	SET_MASK(pComPort->ULCR, LCR_DLAB);
	pComPort->UDLL = pBR->baud_udll;
	pComPort->UDLM = pBR->baud_udlm;
	pComPort->UFCR = FCR_FIFO_EN | FCR_RFRST | FCR_XFRST |
					FCR_64B_FIFO_EN | FCR_FIFO_8;
	pComPort->ULCR = LCR_8N1 | (is_enable_parity ? LCR_PEN : 0) |
					(is_event_parity ? LCR_EPS : 0) |
					(is_onestop_bit ? 0 : LCR_STB);
	ISR4  |= BIT(6 + chan);
	SET_MASK(pComPort->UMCR, MCR_OUT2);
}

/**
 *
 * get_br_idx
 * get baud rate index
 * @return
 *	STATUS_SUCCESS - input baud rate is match supported type
 *	STATUS_WARN_BAUDRATE - input baud rate isn't match, choice the ceiling
 * @parameter
 *	baud_rate - baud rate from dts
 *	baud_rate_index - return the baud rate index
 */
static int get_br_idx(uint32_t baud_rate, uint32_t *baud_rate_index)
{
	int i = 0;

	for (i = 0; i < t_br_max_idx; i++) {
		if (baud_rate <= baudrate_table[i].baud_rate) {
			*baud_rate_index = baudrate_table[i].index;
			if (baud_rate != baudrate_table[i].baud_rate) {
				return STATUS_WARN_BAUDRATE;
			}
			return STATUS_SUCCESS;
		}
	}
	*baud_rate_index = baudrate_table[t_br_max_idx-1].index;
	return STATUS_WARN_BAUDRATE;
}

#define DEV_CFG(dev)					      \
	((const struct uart_ite_it8xxx2_device_config *const) \
	 (dev)->config->config)
#define DEV_DATA(dev) \
	((struct uart_ite_it8xxx2_dev_data_t *)(dev)->data)
struct uart_ite_it8xxx2_device_config {
	uint32_t sys_clk_freq;
};
/** Device data structure */
struct uart_ite_it8xxx2_dev_data_t {
	uint32_t base;
	uint32_t port;
	uint32_t baud_rate;     /**< Baud rate */
	uint32_t irq_no;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN

	/* cache of IIR read clear */
	uint8_t iir_cache;

	/* Callback function pointer */
	uart_irq_callback_user_data_t cb;

	/* Callback function arg */
	void *cb_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static int uart_ite_it8xxx2_poll_in(const struct device *dev, unsigned char *c)
{
	int ret = STATUS_SUCCESS;
	struct uart_ite_it8xxx2_dev_data_t *const dev_data = DEV_DATA(dev);

	ret = uart_getc(dev_data->port, c);
	return ret;
}

static void uart_ite_it8xxx2_poll_out(const struct device *dev, unsigned char c)
{
	struct uart_ite_it8xxx2_dev_data_t *const dev_data = DEV_DATA(dev);

	uart_putc(dev_data->port, c);
}

static int uart_ite_it8xxx2_err_check(const struct device *dev)
{
	struct uart_ite_it8xxx2_dev_data_t *const dev_data = DEV_DATA(dev);

	return uart_is_xmit_empty(dev_data->port);
}

#if CONFIG_UART_INTERRUPT_DRIVEN
/**
 * @brief Fill FIFO with data
 *
 * @param dev UART device struct
 * @param tx_data Data to transmit
 * @param size Number of bytes to send
 *
 * @return Number of bytes sent
 */
static int uart_ite_it8xxx2_fifo_fill(
	const struct device *dev, const uint8_t *tx_data, int size)
{
	int i, val = 0;
	unsigned char ucTemp;
	struct uart_ite_it8xxx2_dev_data_t *const dev_data = DEV_DATA(dev);
	volatile struct T_ITEUART *pComPort =
		(volatile struct T_ITEUART *)dev_data->base;

	if ((dev_data->port == T_UART_2) && DEF_UART2_ENABLE_CTS_RTS) {
		do {
			ucTemp = pComPort->UMSR;
			ucTemp &= 0xf0;
		} while (!(ucTemp & MSR_CTS));
		ucTemp = pComPort->UMSR;
	}
	if ((dev_data->port == T_UART_1) && DEF_UART1_ENABLE_CTS_RTS) {
		do {
			ucTemp = pComPort->UMSR;
			ucTemp &= 0xf0;
		} while (!(ucTemp & MSR_CTS));
		ucTemp = pComPort->UMSR;
	}
	for (i = 0; i < size; i++) {
		if (uart_putc(dev_data->port, tx_data[i]) == STATUS_SUCCESS) {
			val++;
		}
		ucTemp = pComPort->UMSR;
	}
	if ((dev_data->port == T_UART_2) && DEF_UART2_ENABLE_CTS_RTS) {
		do {
			ucTemp = pComPort->UMSR;
			ucTemp &= 0xf0;
		} while (ucTemp != MSR_CTS);
		ucTemp = pComPort->UMSR;
	}
	if ((dev_data->port == T_UART_1) && DEF_UART1_ENABLE_CTS_RTS) {
		do {
			ucTemp = pComPort->UMSR;
			ucTemp &= 0xf0;
		} while (ucTemp != MSR_CTS);
		ucTemp = pComPort->UMSR;
	}
	return val;
}

/**
 * @brief Read data from FIFO
 *
 * @param dev UART device struct
 * @param rxData Data container
 * @param size Container size
 *
 * @return Number of bytes read
 */
static int uart_ite_it8xxx2_fifo_read(const struct device *dev,
				      uint8_t *rx_data, const int size)
{
	int i, val = 0;
	unsigned char ucTemp;
	struct uart_ite_it8xxx2_dev_data_t *const dev_data = DEV_DATA(dev);
	volatile struct T_ITEUART *pComPort =
		(volatile struct T_ITEUART *)dev_data->base;

	if ((dev_data->port == T_UART_2) && DEF_UART2_ENABLE_CTS_RTS) {
		do {
			ucTemp = pComPort->UMSR;
			ucTemp &= 0xf0;
		} while (ucTemp != MSR_CTS);
		ucTemp = pComPort->UMSR;
	}
	if ((dev_data->port == T_UART_1) && DEF_UART1_ENABLE_CTS_RTS) {
		do {
			ucTemp = pComPort->UMSR;
			ucTemp &= 0xf0;
		} while (ucTemp != MSR_CTS);
		ucTemp = pComPort->UMSR;
	}
	for (i = 0; i < size; i++) {
		if (uart_getc(dev_data->port, &rx_data[i]) == STATUS_SUCCESS) {
			val++;
		}
	}
	ucTemp = pComPort->UMSR;
	return val;
}

static inline volatile struct T_ITEUART*
	get_iteuart_port(const struct device *dev)
{
	struct uart_ite_it8xxx2_dev_data_t *const dev_data = DEV_DATA(dev);

	return (volatile struct T_ITEUART *)dev_data->base;
}

/**
 * @brief Enable TX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_ite_it8xxx2_irq_tx_enable(const struct device *dev)
{
	volatile struct T_ITEUART *pComPort = get_iteuart_port(dev);

	SET_MASK(pComPort->UIER, IER_ETHREI);
}

/**
 * @brief Disable TX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_ite_it8xxx2_irq_tx_disable(const struct device *dev)
{
	volatile struct T_ITEUART *pComPort = get_iteuart_port(dev);

	CLEAR_MASK(pComPort->UIER, IER_ETHREI);
}

/**
 * @brief Check if Tx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_ite_it8xxx2_irq_tx_ready(const struct device *dev)
{
	int ret = STATUS_SUCCESS;

	if ((IIRC(dev) & IIR_MASK_ID) == IIR_TX_HOLD_EMPTY) {
		ret = STATUS_IRQ_READY;
	}
	return ret;
}

/**
 * @brief Check if nothing remains to be transmitted
 *
 * @param dev UART device struct
 *
 * @return 1 if nothing remains to be transmitted, 0 otherwise
 */
static int uart_ite_it8xxx2_irq_tx_complete(const struct device *dev)
{
	uint8_t mask;
	volatile struct T_ITEUART *pComPort = get_iteuart_port(dev);

	mask = (LSR_TEMT | LSR_THRE);
	return ((pComPort->ULSR & mask) == mask);
}

/**
 * @brief Enable RX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_ite_it8xxx2_irq_rx_enable(const struct device *dev)
{
	volatile struct T_ITEUART *pComPort = get_iteuart_port(dev);

	SET_MASK(pComPort->UIER, IER_ERDVI);
}

/**
 * @brief Disable RX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_ite_it8xxx2_irq_rx_disable(const struct device *dev)
{
	volatile struct T_ITEUART *pComPort = get_iteuart_port(dev);

	CLEAR_MASK(pComPort->UIER, IER_ERDVI);
}

/**
 * @brief Check if Rx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_ite_it8xxx2_irq_rx_ready(const struct device *dev)
{
	int ret = STATUS_SUCCESS;

	if ((IIRC(dev) & IIR_MASK_ID) == IIR_RECV_AVAILABLE) {
		ret = STATUS_IRQ_READY;
	}
	return ret;
}

/**
 * @brief Enable error interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_ite_it8xxx2_irq_err_enable(const struct device *dev)
{
	volatile struct T_ITEUART *pComPort = get_iteuart_port(dev);

	SET_MASK(pComPort->UIER, IER_ERLSI);
}

/**
 * @brief Disable error interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static void uart_ite_it8xxx2_irq_err_disable(const struct device *dev)
{
	volatile struct T_ITEUART *pComPort = get_iteuart_port(dev);

	CLEAR_MASK(pComPort->UIER, IER_ERLSI);
}

/**
 * @brief Check if any IRQ is pending
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is pending, 0 otherwise
 */
static int uart_ite_it8xxx2_irq_is_pending(const struct device *dev)
{
	return (!(IIRC(dev) & IIR_NONE));
}

/**
 * @brief Update cached contents of IIR
 *
 * @param dev UART device struct
 *
 * @return Always 1
 */
static int uart_ite_it8xxx2_irq_update(const struct device *dev)
{
	volatile struct T_ITEUART *pComPort = get_iteuart_port(dev);

	IIRC(dev) = pComPort->UIIR;
	return STATUS_IRQ_READY;
}

/**
 * @brief Set the callback function pointer for IRQ.
 *
 * @param dev UART device struct
 * @param cb Callback function pointer.
 *
 * @return N/A
 */
static void uart_ite_it8xxx2_irq_callback_set(const struct device *dev,
					      uart_irq_callback_user_data_t cb,
					      void *cb_data)
{
	struct uart_ite_it8xxx2_dev_data_t *const dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 *
 * @return N/A
 */
static void uart_ite_it8xxx2_isr(const void *arg)
{
	const struct device *dev = arg;
	struct uart_ite_it8xxx2_dev_data_t *const dev_data = DEV_DATA(dev);

	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}

}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

/**
 * @brief Initialize individual UART port
 *
 * This routine is called to reset the chip in a quiescent state.
 *
 * @param dev UART device struct
 *
 * @return 0 if successful, failed otherwise
 */
static int uart_ite_it8xxx2_init(const struct device *dev)
{
	int ret = STATUS_SUCCESS;
	uint32_t br_index;
	struct uart_ite_it8xxx2_dev_data_t *const dev_data = DEV_DATA(dev);
	unsigned int old_level = irq_lock();

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	dev_data->iir_cache = 0U;
#endif
	ret = get_br_idx(dev_data->baud_rate, &br_index);
	dev_data->port =
		(dev_data->base == (uint32_t)mComPortTb[0].mUartBase) ? 0 : 1;
	uart_init(dev_data->port, br_index, false, false, true);
	irq_unlock(old_level);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	irq_connect_dynamic(dev_data->irq_no, 0, uart_ite_it8xxx2_isr, dev, 0);
	irq_enable(dev_data->irq_no);
#endif

	return ret;
}

static const struct uart_driver_api uart_ite_it8xxx2_driver_api = {
	.poll_in = uart_ite_it8xxx2_poll_in,
	.poll_out = uart_ite_it8xxx2_poll_out,
	.err_check = uart_ite_it8xxx2_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_ite_it8xxx2_fifo_fill,
	.fifo_read = uart_ite_it8xxx2_fifo_read,
	.irq_tx_enable = uart_ite_it8xxx2_irq_tx_enable,
	.irq_tx_disable = uart_ite_it8xxx2_irq_tx_disable,
	.irq_tx_ready = uart_ite_it8xxx2_irq_tx_ready,
	.irq_tx_complete = uart_ite_it8xxx2_irq_tx_complete,
	.irq_rx_enable = uart_ite_it8xxx2_irq_rx_enable,
	.irq_rx_disable = uart_ite_it8xxx2_irq_rx_disable,
	.irq_rx_ready = uart_ite_it8xxx2_irq_rx_ready,
	.irq_err_enable = uart_ite_it8xxx2_irq_err_enable,
	.irq_err_disable = uart_ite_it8xxx2_irq_err_disable,
	.irq_is_pending = uart_ite_it8xxx2_irq_is_pending,
	.irq_update = uart_ite_it8xxx2_irq_update,
	.irq_callback_set = uart_ite_it8xxx2_irq_callback_set,
#endif
};

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay)
static const struct uart_ite_it8xxx2_device_config
	uart_ite_it8xxx2_dev_cfg_0 = {
	.sys_clk_freq = DT_PROP(DT_NODELABEL(uart1), clock_frequency),
};
static struct uart_ite_it8xxx2_dev_data_t
	uart_ite_it8xxx2_dev_data_0 = {
	.base = DT_REG_ADDR_BY_IDX(DT_NODELABEL(uart1), 0),
	.baud_rate = DT_PROP(DT_NODELABEL(uart1), current_speed),

	#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_no = DT_IRQ_BY_IDX(DT_NODELABEL(uart1), 0, irq),
	#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};
DEVICE_AND_API_INIT(uart_ite_it8xxx2_0,
		    DT_PROP(DT_NODELABEL(uart1), label), &uart_ite_it8xxx2_init,
		    &uart_ite_it8xxx2_dev_data_0, &uart_ite_it8xxx2_dev_cfg_0,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_ite_it8xxx2_driver_api);

#endif /* uart1 devicetree is okay */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay)
static const struct uart_ite_it8xxx2_device_config
	uart_ite_it8xxx2_dev_cfg_1 = {
	.sys_clk_freq = DT_PROP(DT_NODELABEL(uart2), clock_frequency),
};
static struct uart_ite_it8xxx2_dev_data_t
	uart_ite_it8xxx2_dev_data_1 = {
	.base = DT_REG_ADDR_BY_IDX(DT_NODELABEL(uart2), 0),
	.baud_rate = DT_PROP(DT_NODELABEL(uart2), current_speed),

	#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_no = DT_IRQ_BY_IDX(DT_NODELABEL(uart2), 0, irq),
	#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};
DEVICE_AND_API_INIT(uart_ite_it8xxx2_1,
		    DT_PROP(DT_NODELABEL(uart2), label), &uart_ite_it8xxx2_init,
		    &uart_ite_it8xxx2_dev_data_1, &uart_ite_it8xxx2_dev_cfg_1,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_ite_it8xxx2_driver_api);
#endif /* uart2 devicetree is okay */
