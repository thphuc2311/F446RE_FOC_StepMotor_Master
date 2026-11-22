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
	if (!IsWaitingRequestCAN || send_entry == NULL || canCmd != send_entry->canCmd)
	{
		return;
	}

	memset(RxCANData2usb, 0, sizeof(RxCANData2usb));
	memcpy(RxCANData2usb, data, (len < FRAME_DATA_LEN) ? len : FRAME_DATA_LEN);
	NodeID_Info = node;
	OnCANFreshData_b = true;
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

// standard data frame
// [SOF]     [NodeId]    [CMD]   [TypeCmd]   [8 Bytes Data]  [Checksum]
// [0xA5]    [0x01]      [0x23]  [0x01]      [8 Bytes Data]  [Checksum]
// [1Byte]   [1Byte]     [1Byte]             [8Bytes]        [2Byte]     = 13 bytes tota

// dynamic data frame
// [SOF]     [NodeId_1]  [CMD]   [8 Bytes Data]  +    [NodeId_2]  [CMD]   [8 Bytes Data]  [Checksum]
// [0xA5]    [0x01]      [0x23]  [8 Bytes Data]  +    [0x02]      [0x23]  [8 Bytes Data]  [Checksum]
// [1Byte]   [1Byte]     [1Byte] [8Bytes]        +    [1Byte]     [1Byte] [8Bytes]        [2Byte]
// 22 bytes total = 1 byte SOF + 2*(1+1+1+8) + 2 byte checksum
// number CAN frames in dynamic data frame = (total bytes - 1 byte SOF - 2 byte checksum) / 13 bytes

