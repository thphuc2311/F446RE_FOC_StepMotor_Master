#ifndef __FRAME_H__
#define __FRAME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * Frame layout (13 bytes):
 *   [0]     start byte, fixed 0xA5
 *   [1]     node ID (CAN node the data belongs to)
 *   [2]     command   (FRAME_CMD_x)
 *   [3]     data type (FRAME_TYPE_x)
 *   [4..11] 8 data bytes
 *   [12]    checksum = (sum of bytes 0..11) & 0xFF
 */
#define FRAME_START_BYTE		0xA5u
#define FRAME_ERROR_BYTE		0xEEu
#define FRAME_ERROR_FLAG		0xFFu
#define FRAME_DATA_LEN		8u
#define FRAME_LEN			13u

#define FRAME_IDX_START		0u
#define FRAME_IDX_NODE		1u
#define FRAME_IDX_CMD		2u
#define FRAME_IDX_TYPE		3u
#define FRAME_IDX_DATA		4u
#define FRAME_IDX_CHECKSUM	12u

typedef enum
{
	FRAME_CMD_SET	= 1,
	FRAME_CMD_GET	= 2,
	FRAME_CMD_STORE	= 3
} FrameCmd_t;

typedef enum
{
	FRAME_TYPE_POS		= 1,
	FRAME_TYPE_VEL		= 2,
	FRAME_TYPE_CURLIM	= 3,
	FRAME_TYPE_TEMP		= 4
} FrameType_t;

#define FRAME_STREAM_PERIOD_MS	10u
#define FRAME_RESP_TIMEOUT_MS	100u

uint8_t Frame_Checksum(const uint8_t *frame);
void Frame_Build(uint8_t *frame, uint8_t node, uint8_t cmd, uint8_t type, const uint8_t *data);

/* Parse USB text command "USB get <pos|vel|curlim|temp> <1FRAME|STREAM>". */
void Frame_UsbRxParseCmd(void);
/* Issue slave requests and send frames over USB. Call from main loop. */
void Frame_Task(void);
/* Call from OnCanCmdResponded() with the responding node and raw slave payload. */
void Frame_OnCanResponse(uint8_t node, uint8_t canCmd, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __FRAME_H__ */
