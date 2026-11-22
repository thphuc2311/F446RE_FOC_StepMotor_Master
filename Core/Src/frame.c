#include "frame.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
	const char *name;
	uint8_t canCmd;
	void (*request)(void);
	HandleMode_t mode;
} FrameGetEntry_t;

typedef struct
{
	const char   *name;
	uint8_t       canCmd;
	const uint8_t numparam;
} FrameSetEntry_t;

static FrameGetEntry_t USB_GetTable[] =
{
	{ "pos",    0x23, UpdateAngle,     ENTRY_IDLE },
	{ "vel",    0x22, GetVelocity,     ENTRY_IDLE },
	{ "curlim", 0x25, GetCurrentLimit, ENTRY_IDLE },
	{ "temp",   0x26, GetTemperature,  ENTRY_IDLE },
};
#define GET_TABLE_SIZE	(sizeof(USB_GetTable) / sizeof(USB_GetTable[0]))

static FrameSetEntry_t USB_SetTable[] =
{
	{ "cur",    	0x03, 1 }, // SetCurrentSetPoint
	{ "vel",    	0x04, 1 }, // SetVelocitySetPoint
	{ "pos", 		0x05, 2 }, // SetPositionSetPoint
	{ "postime",	0x06, 2 }, // SetPositionSetPointWithTime
	{ "curlim", 	0x12, 2 }, // SetCurrentLimit
};
#define SET_TABLE_SIZE	(sizeof(USB_SetTable) / sizeof(USB_SetTable[0]))

/* --- UART tables (1-shot: 1 request → 1 response, no streaming) --- */
static FrameGetEntry_t UART_GetTable[] =
{
	{ "pos",    0x23, UpdateAngle,     ENTRY_IDLE },
	{ "vel",    0x22, GetVelocity,     ENTRY_IDLE },
	{ "curlim", 0x25, GetCurrentLimit, ENTRY_IDLE },
	{ "temp",   0x26, GetTemperature,  ENTRY_IDLE },
};
#define UART_GET_TABLE_SIZE  (sizeof(UART_GetTable) / sizeof(UART_GetTable[0]))

static FrameSetEntry_t UART_SetTable[] =
{
	{ "cur",     	0x03, 1 }, // SetCurrentSetPoint
	{ "vel",     	0x04, 1 }, // SetVelocitySetPoint
	{ "pos", 		0x05, 2 }, // SetPositionSetPoint
	{ "postime",	0x06, 2 }, // SetPositionSetPointWithTime
	{ "curlim", 	0x12, 2 }, // SetCurrentLimit
};
#define UART_SET_TABLE_SIZE  (sizeof(UART_SetTable) / sizeof(UART_SetTable[0]))

/* UART 1-shot state */
static FrameGetEntry_t *uart_send_entry = NULL;
static bool uart_waitingCAN = false;
static uint8_t uart_RxCANData[FRAME_DATA_LEN];
static volatile uint8_t uart_NodeID = 0;
static volatile bool uart_CANFreshData = false;

static FrameGetEntry_t *send_entry = NULL;
static bool IsWaitingRequestCAN = false;       /* true while waiting for CAN response */
static uint8_t s_streamIndex = 0;    /* round-robin index for MODE_STREAM */

static uint8_t RxCANData2usb[FRAME_DATA_LEN];
static volatile uint8_t NodeID_Info = 0;

static uint8_t usb_txFrame[FRAME_LEN];

static uint8_t usb_ErrorFrame[FRAME_LEN];
static bool ErrorFrameInOrder_b = false;

static volatile bool OnCANFreshData_b = false;

/*
 * Queue an error frame to be sent over USB.
 * Error  only used to notify to PC application when an invalid command is received.
 * Error frame for slave CAN node not yet implemented.
 */
static void Frame_QueueError(void)
{
	memset(usb_ErrorFrame, 0, sizeof(usb_ErrorFrame));
	usb_ErrorFrame[FRAME_IDX_START] = FRAME_ERROR_BYTE;
	usb_ErrorFrame[FRAME_IDX_DATA] = FRAME_ERROR_FLAG;
	ErrorFrameInOrder_b = true;
}

uint8_t Frame_Checksum(const uint8_t *frame)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < FRAME_IDX_CHECKSUM; i++)
	{
		sum += frame[i];
	}
	return sum;
}

void Frame_Build(uint8_t *frame, uint8_t node, uint8_t cmd, const uint8_t *data)
{
	frame[FRAME_IDX_START] = FRAME_START_BYTE;
	frame[FRAME_IDX_NODE] = node;
	frame[FRAME_IDX_CMD] = cmd;
	memcpy(&frame[FRAME_IDX_DATA], data, FRAME_DATA_LEN);
	frame[FRAME_IDX_CHECKSUM] = Frame_Checksum(frame);
}

static bool Frame_ExecuteSet(const char *param, const char *value)
{
	/* Find matching entry by name */
	const FrameSetEntry_t *entry = NULL;
	for (uint32_t i = 0; i < SET_TABLE_SIZE; i++)
	{
		if (strcmp(param, USB_SetTable[i].name) == 0)
		{
			entry = &USB_SetTable[i];
			break;
		}
	}
	if (!entry)
		return false;

	/* Parse params: comma-separated, e.g. "1000,1" or "1000" */
	int32_t params[2] = {0};
	uint8_t numParsed = 0;
	char buf[64];
	strncpy(buf, value, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	char *saveptr = NULL;
	char *tok = strtok_r(buf, ",", &saveptr);
	while (tok && numParsed < entry->numparam)
	{
		errno = 0;
		char *endptr = NULL;
		long v = strtol(tok, &endptr, 10);
		if (endptr == tok || *endptr != '\0' || errno == ERANGE)
			return false;
		params[numParsed++] = (int32_t)v;
		tok = strtok_r(NULL, ",", &saveptr);
	}

	if (numParsed != entry->numparam)
		return false;

	/* Dispatch directly — no wrapper, no function pointer */
	switch (entry->canCmd)
	{
		case 0x03:  SetCurrentSetPoint(params[0]);                          break;
		case 0x04:  SetVelocitySetPoint(params[0]);                       break;
		case 0x05:  SetPositionSetPoint(params[0], (bool)params[1]);      break;
		case 0x06:  SetPositionSetPointWithTime(params[0], (float)params[1]); break;
		case 0x12:  SetCurrentLimit(params[0], (bool)params[1]);           break;
		default:    return false;
	}
	return true;
}

void Frame_UsbRxParseCmd(void)
{
	char line[64];
	uint32_t len;

	if (recvDone == 0)
	{
		return;
	}

	len = recvLen;
	if (len >= sizeof(line))
	{
		recvDone = 0;
		Frame_QueueError();
		return;
	}
	memcpy(line, UserRxBufferFS, len);
	line[len] = '\0';
	recvDone = 0;

	/* Reject embedded NUL bytes rather than executing a partial command. */
	if (memchr(line, '\0', len) != NULL)
	{
		Frame_QueueError();
		return;
	}

	char *saveptr = NULL;
	char *tag      = strtok_r(line, " \t\r\n", &saveptr);
	char *action   = strtok_r(NULL, " \t\r\n", &saveptr);
	char *param    = strtok_r(NULL, " \t\r\n", &saveptr);
	char *argument = strtok_r(NULL, " \t\r\n", &saveptr);
	char *extra    = strtok_r(NULL, " \t\r\n", &saveptr);

	if (!tag || !action || !param || !argument || extra ||
	    strcmp(tag, "USB") != 0)
	{
		Frame_QueueError();
		return;
	}

	if (strcmp(action, "set") == 0)
	{
		if (!Frame_ExecuteSet(param, argument))
		{
			Frame_QueueError();
		}
		/* No USB success frame: CAN submission is not a slave acknowledgment.
		 * A valid set command leaves an existing get stream running. */
		return;
	}

	if (strcmp(action, "get") != 0)
	{
		Frame_QueueError();
		return;
	}

	/* Stop streaming for the specified entry only. */
	if (strcmp(argument, "STOP") == 0)
	{
		for (uint32_t i = 0; i < GET_TABLE_SIZE; i++)
		{
			if (strcmp(param, USB_GetTable[i].name) == 0)
			{
				USB_GetTable[i].mode = ENTRY_IDLE;
				break;
			}
		}
		IsWaitingRequestCAN = false;
		return;
	}

	HandleMode_t newMode;
	if (strcmp(argument, "1FRAME") == 0)
	{
		newMode = ENTRY_ONCE;
	}
	else if (strcmp(argument, "STREAM") == 0)
	{
		newMode = ENTRY_STREAM;
	}
	else
	{
		Frame_QueueError();
		return;
	}

	for (uint32_t i = 0; i < GET_TABLE_SIZE; i++)
	{
		if (strcmp(param, USB_GetTable[i].name) == 0)
		{
			USB_GetTable[i].mode = newMode;
			return;
		}
	}

	Frame_QueueError();
}

void Frame_OnCanResponse(uint8_t node, uint8_t canCmd, const uint8_t *data, uint32_t len)
{
	if (IsWaitingRequestCAN && send_entry != NULL && canCmd == send_entry->canCmd)
	{
		memset(RxCANData2usb, 0, sizeof(RxCANData2usb));
		memcpy(RxCANData2usb, data, (len < FRAME_DATA_LEN) ? len : FRAME_DATA_LEN);
		NodeID_Info = node;
		OnCANFreshData_b = true;
	}

	/* UART 1-shot response */
	if (uart_waitingCAN && uart_send_entry != NULL && canCmd == uart_send_entry->canCmd)
	{
		memset(uart_RxCANData, 0, sizeof(uart_RxCANData));
		memcpy(uart_RxCANData, data, (len < FRAME_DATA_LEN) ? len : FRAME_DATA_LEN);
		uart_NodeID = node;
		uart_CANFreshData = true;
	}
}

void Frame_Task(void)
{
	/* --- Priority 1: send error frame if pending --- */
	if (ErrorFrameInOrder_b && !isTxBusy)
	{
		if (CDC_Transmit_FS(usb_ErrorFrame, FRAME_LEN) == USBD_OK)
		{
			isTxBusy = true;
			ErrorFrameInOrder_b = false;
		}
		return;
	}

	/* --- Priority 2: if CAN response arrived, build & send USB frame --- */
	if (OnCANFreshData_b && !isTxBusy)
	{
		OnCANFreshData_b = false;
		IsWaitingRequestCAN = false;

		/* Entry was stopped while its request was in flight. */
		if (send_entry->mode == ENTRY_IDLE)
		{
			return;
		}

		Frame_Build(usb_txFrame, NodeID_Info, send_entry->canCmd, RxCANData2usb);
		if (CDC_Transmit_FS(usb_txFrame, FRAME_LEN) == USBD_OK)
		{
			isTxBusy = true;
		}

		/* ENTRY_ONCE: one-shot done, demote to IDLE */
		if (send_entry->mode == ENTRY_ONCE)
		{
			send_entry->mode = ENTRY_IDLE;
		}
		return;
	}

	/* --- Priority 3: if not waiting and USB is free, fire next CAN request --- */
	if (isTxBusy || IsWaitingRequestCAN)
	{
		return;
	}

	/* s_streamIndex always points at the next candidate, so each entry is served in turn. */
	for (uint32_t n = 0; n < GET_TABLE_SIZE; n++)
	{
		FrameGetEntry_t *entry = &USB_GetTable[s_streamIndex];
		if (++s_streamIndex >= GET_TABLE_SIZE)
			s_streamIndex = 0;

		if (entry->mode != ENTRY_IDLE)
		{
			send_entry = entry;
			send_entry->request();
			IsWaitingRequestCAN = true;
			return;
		}
	}
}

/* ------------------------------------------------------------------ */
/*  UART command parser — 1-shot: 1 request → 1 response, no stream   */
/* ------------------------------------------------------------------ */

static bool Uart_ExecuteSet(const char *param, const char *value)
{
	const FrameSetEntry_t *entry = NULL;
	for (uint32_t i = 0; i < UART_SET_TABLE_SIZE; i++)
	{
		if (strcmp(param, UART_SetTable[i].name) == 0)
		{
			entry = &UART_SetTable[i];
			break;
		}
	}
	if (!entry)
		return false;

	int32_t params[2] = {0};
	uint8_t numParsed = 0;
	char buf[64];
	strncpy(buf, value, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	char *saveptr = NULL;
	char *tok = strtok_r(buf, ",", &saveptr);
	while (tok && numParsed < entry->numparam)
	{
		errno = 0;
		char *endptr = NULL;
		long v = strtol(tok, &endptr, 10);
		if (endptr == tok || *endptr != '\0' || errno == ERANGE)
			return false;
		params[numParsed++] = (int32_t)v;
		tok = strtok_r(NULL, ",", &saveptr);
	}
	if (numParsed != entry->numparam)
		return false;

	switch (entry->canCmd)
	{
		case 0x03:  SetCurrentSetPoint(params[0]);                          break;
		case 0x04:  SetVelocitySetPoint(params[0]);                       break;
		case 0x05:  SetPositionSetPoint(params[0], (bool)params[1]);      break;
		case 0x06:  SetPositionSetPointWithTime(params[0], (float)params[1]); break;
		case 0x12:  SetCurrentLimit(params[0], (bool)params[1]);           break;
		default:    return false;
	}
	return true;
}

static bool Uart_ExecuteGet(const char *param)
{
	for (uint32_t i = 0; i < UART_GET_TABLE_SIZE; i++)
	{
		if (strcmp(param, UART_GetTable[i].name) == 0)
		{
			uart_send_entry = &UART_GetTable[i];
			uart_send_entry->request();
			uart_waitingCAN = true;
			return true;
		}
	}
	return false;
}

bool Frame_UartParseCmd(char *line)
{
	char *saveptr = NULL;
	char *action = strtok_r(line, " \t\r\n", &saveptr);
	char *param  = strtok_r(NULL, " \t\r\n", &saveptr);
	char *value  = strtok_r(NULL, "\r\n", &saveptr);  /* rest of line = value */
	char *extra  = value ? strtok_r(NULL, "\r\n", &saveptr) : NULL;

	if (!action || !param || extra)
		return false;

	/* trim trailing whitespace from value */
	if (value)
	{
		size_t vlen = strlen(value);
		while (vlen > 0 && (value[vlen-1] == ' ' || value[vlen-1] == '\t'))
			value[--vlen] = '\0';
	}

	if (strcmp(action, "set") == 0)
	{
		if (!value || !Uart_ExecuteSet(param, value))
			return false;
		return true;
	}

	if (strcmp(action, "get") == 0)
	{
		if (!Uart_ExecuteGet(param))
			return false;
		return true;
	}

	return false;
}

void Frame_UartTask(void)
{
	if (!uart_CANFreshData || !uart_waitingCAN)
		return;

	uart_CANFreshData = false;
	uart_waitingCAN = false;

	/* Format response over UART */
	if (uart_send_entry->canCmd == 0x26)
	{
		/* Temperature: 2 floats */
		float mcu, ntc;
		memcpy(&mcu, uart_RxCANData, sizeof(float));
		memcpy(&ntc, uart_RxCANData + 4, sizeof(float));
		snprintf((char *)bufferTxUART, MAX_BUFFER_SIZE,
			"get %s Cmd OK MCU:%.3f NTC:%.3f\n",
			uart_send_entry->name, mcu, ntc);
	}
	else
	{
		/* Generic: 1 int32 + state byte */
		int32_t val = 0;
		memcpy(&val, uart_RxCANData, sizeof(int32_t));
		snprintf((char *)bufferTxUART, MAX_BUFFER_SIZE,
			"get %s %ld Cmd OK\n",
			uart_send_entry->name, val);
	}

	HAL_UART_Transmit_DMA(&huart2, bufferTxUART, strlen((char *)bufferTxUART));
}

// standard data frame
// [SOF]     [NodeId]    [CMD]   [8 Bytes Data]  [Checksum]
// [0xA5]    [0x01]      [0x23]  [8 Bytes Data]  [Checksum]
// [1Byte]   [1Byte]     [1Byte] [8Bytes]        [2Byte]     = 13 bytes total

// dynamic data frame
// [SOF]     [NodeId_1]  [CMD]   [8 Bytes Data]  +    [NodeId_2]  [CMD]   [8 Bytes Data]  [Checksum]
// [0xA5]    [0x01]      [0x23]  [8 Bytes Data]  +    [0x02]      [0x23]  [8 Bytes Data]  [Checksum]
// [1Byte]   [1Byte]     [1Byte] [8Bytes]        +    [1Byte]     [1Byte] [8Bytes]        [2Byte]
// 22 bytes total = 1 byte SOF + 2*(1+1+1+8) + 2 byte checksum
// number CAN frames in dynamic data frame = (total bytes - 1 byte SOF - 2 byte checksum) / 13 bytes

