#ifndef __FRAME_H__
#define __FRAME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * Frame layout (12 bytes):
 *   [0]     start byte, fixed 0xA5
 *   [1]     node ID (CAN node the data belongs to)
 *   [2]     CAN command of the slave response (e.g. 0x23 = position)
 *   [3..10] 8 data bytes
 *   [11]    checksum = (sum of bytes 0..10) & 0xFF
 */
#define FRAME_START_BYTE		0xA5u
#define FRAME_ERROR_BYTE		0xEEu
#define FRAME_ERROR_FLAG		0xFFu
#define FRAME_DATA_LEN		8u
#define FRAME_LEN			12u

#define FRAME_IDX_START		0u
#define FRAME_IDX_NODE		1u
#define FRAME_IDX_CMD		2u
#define FRAME_IDX_DATA		3u
#define FRAME_IDX_CHECKSUM	11u

#define FRAME_STREAM_PERIOD_MS	10u
#define FRAME_RESP_TIMEOUT_MS	100u

uint8_t Frame_Checksum(const uint8_t *frame);
void Frame_Build(uint8_t *frame, uint8_t node, uint8_t cmd, const uint8_t *data);

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
