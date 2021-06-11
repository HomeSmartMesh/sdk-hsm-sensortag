#include <stdint.h>


#define MESH_Broadcast_Header_Length 4
#define MESH_P2P_Header_Length 5

#define MAX_MESH_MESSAGE_SIZE (CONFIG_ESB_MAX_PAYLOAD_LENGTH-MESH_P2P_Header_Length)

#define Mesh_Pid_Text       0x16


//------------------------- Mesh Macros -------------------------

#define MESH_IS_BROADCAST(val) ((val & 0x80) == 0x80)
#define MESH_IS_PEER2PEER(val) ((val & 0x80) == 0x00)
//Ack if bits 1,2 == 1,0 => MASK 0x60, VAL 0x40
#define MESH_IS_ACKNOWLEDGE(val) ((val & 0x60) == 0x40)
#define MESH_WANT_ACKNOWLEDGE(val) ((val & 0xF0) == 0x70)
#define MESH_IS_RESPONSE(val) ((val & 0xF0) == 0x00)

typedef struct 
{
    uint8_t control;
    uint8_t pid;
    uint8_t source;
    uint8_t dest;
    int8_t  rssi;       //Radio Signal Strength Indication
    uint8_t payload_length;
    uint8_t *payload;
}message_t;

void sm_start();
void mesh_bcast_text(char *text);
