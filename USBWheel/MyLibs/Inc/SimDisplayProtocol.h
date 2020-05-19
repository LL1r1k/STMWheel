#ifndef INC_SIMDISPLAYPROTOCOL_H_
#define INC_SIMDISPLAYPROTOCOL_H_

#define SIMDISPLAYPROTOCOL_VERSION "1"

struct SimDisplayPacket {
	uint8_t id;
	uint8_t status;
	uint16_t rpm;
	uint16_t optrpm;
	uint16_t shftrpm;
	uint8_t ebit;
};

#define SDP_STATUS_OFF 0
#define SDP_STATUS_REPLAY 1
#define SDP_STATUS_LIVE 2
#define SDP_STATUS_PAUSE 3

#endif /* INC_SIMDISPLAYPROTOCOL_H_ */
