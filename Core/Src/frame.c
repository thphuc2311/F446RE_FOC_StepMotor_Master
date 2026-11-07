#include "frame.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>

void GetVelocity(void);
void UpdateAngle(void);
void GetCurrentLimit(void);
void GetTemperature(void);

typedef struct
{
	const char *name;
	uint8_t canCmd;
	void (*request)(void);
} FrameGetEntry_t;

static const FrameGetEntry_t s_getTable[] =
{
	{ "pos",    0x23, UpdateAngle      },
	{ "vel",    0x22, GetVelocity      },
	{ "curlim", 0x25, GetCurrentLimit  },
	{ "temp",   0x26, GetTemperature   },
};
#define GET_TABLE_SIZE	(sizeof(s_getTable) / sizeof(s_getTable[0]))

typedef enum { MODE_OFF, MODE_ONCE, MODE_STREAM } Mode_t;

static Mode_t s_mode = MODE_OFF;
static const FrameGetEntry_t *s_entry = NULL;
static bool s_waiting = false;
static uint32_t s_tReq = 0;
static uint32_t s_tLastFrame = 0;

static volatile bool s_respReady = false;
static uint8_t s_respData[FRAME_DATA_LEN];
static volatile uint8_t s_respNode = 0;

static uint8_t s_txFrame[FRAME_LEN];
static bool s_txPending = false;

static uint8_t s_errFrame[FRAME_LEN];
static bool s_errPending = false;

/*
 * Queue an error frame to be sent over USB.
 * Error  only used to notify to PC application when an invalid command is received.
 * Error frame for slave CAN node not yet implemented.
 */
static void Frame_QueueError(void)
{
	memset(s_errFrame, 0, sizeof(s_errFrame));
	s_errFrame[FRAME_IDX_START] = FRAME_ERROR_BYTE;
	s_errFrame[FRAME_IDX_DATA] = FRAME_ERROR_FLAG;
	s_errPending = true;
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
		len = sizeof(line) - 1;
	}
	memcpy(line, UserRxBufferFS, len);
	line[len] = '\0';
	recvDone = 0;

	char *saveptr = NULL;
	char *tag    = strtok_r(line, " \r\n", &saveptr);
	char *action = strtok_r(NULL, " \r\n", &saveptr);
	char *param  = strtok_r(NULL, " \r\n", &saveptr);
	char *mode   = strtok_r(NULL, " \r\n", &saveptr);
	char *extra  = strtok_r(NULL, " \r\n", &saveptr);

	if (!tag || !action || !param || !mode || extra ||
	    strcmp(tag, "USB") != 0 || strcmp(action, "get") != 0)
	{
		Frame_QueueError();
		return;
	}

	/* Stop an active stream. */
	if (strcmp(mode, "STOP") == 0)
	{
		s_mode = MODE_OFF;
		s_waiting = false;
		s_respReady = false;
		return;
	}

	Mode_t newMode;
	if (strcmp(mode, "1FRAME") == 0)
	{
		newMode = MODE_ONCE;
	}
	else if (strcmp(mode, "STREAM") == 0)
	{
		newMode = MODE_STREAM;
	}
	else
	{
		Frame_QueueError();
		return;
	}

	for (uint32_t i = 0; i < GET_TABLE_SIZE; i++)
	{
		if (strcmp(param, s_getTable[i].name) == 0)
		{
			s_mode = MODE_OFF;
			s_waiting = false;
			s_respReady = false;
			s_entry = &s_getTable[i];
			s_tLastFrame = HAL_GetTick() - FRAME_STREAM_PERIOD_MS;
			s_mode = newMode;
			return;
		}
	}

	Frame_QueueError();
}

void Frame_OnCanResponse(uint8_t node, uint8_t canCmd, const uint8_t *data, uint32_t len)
{
	if (!s_waiting || s_respReady || s_entry == NULL || canCmd != s_entry->canCmd)
	{
		return;
	}

	memset(s_respData, 0, sizeof(s_respData));
	memcpy(s_respData, data, (len < FRAME_DATA_LEN) ? len : FRAME_DATA_LEN);
	s_respNode = node;
	s_respReady = true;
}

void Frame_Task(void)
{
	uint32_t now = HAL_GetTick();

	if (s_errPending)
	{
		if (CDC_Transmit_FS(s_errFrame, FRAME_LEN) == USBD_OK)
		{
			isTxBusy = true;
			s_errPending = false;
		}
		return;
	}

	if (s_mode == MODE_OFF || s_entry == NULL)
	{
		return;
	}

	if (s_txPending)
	{
		if (CDC_Transmit_FS(s_txFrame, FRAME_LEN) == USBD_OK)
		{
			isTxBusy = true;
			s_txPending = false;
			if (s_mode == MODE_ONCE)
			{
				s_mode = MODE_OFF;
			}
		}
		return;
	}

	if (s_waiting)
	{
		if (s_respReady)
		{
			Frame_Build(s_txFrame, s_respNode, s_entry->canCmd, s_respData);
			s_respReady = false;
			s_waiting = false;
			s_tLastFrame = now;
			s_txPending = true;
		}
		else if ((now - s_tReq) >= FRAME_RESP_TIMEOUT_MS)
		{
			s_waiting = false;
			if (s_mode == MODE_ONCE)
			{
				s_mode = MODE_OFF;
			}
		}
		return;
	}

	if (isTxBusy == false && (now - s_tLastFrame) >= FRAME_STREAM_PERIOD_MS)
	{
		s_respReady = false;
		s_waiting = true;
		s_tReq = now;
		s_entry->request();
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

