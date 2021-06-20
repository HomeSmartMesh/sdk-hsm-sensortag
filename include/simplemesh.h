#include <stdint.h>

#ifdef __cplusplus
#include <string>
//https://nlohmann.github.io/json/
#include <json.hpp>
using json = nlohmann::json;
extern "C" {
#endif


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

typedef void (*mesh_rx_handler_t)(message_t*);

void sm_start();
void sm_set_callback_rx_message(mesh_rx_handler_t rx_handler);
void mesh_bcast_text(const char *text);

//------------------------- CPP wrapper interfaces -------------------------
#ifdef __cplusplus
typedef void (*mesh_rx_json_handler_t)(std::string &topic, json &data);
std::string sm_get_uid();
std::string sm_get_topic();
bool is_self(std::string &payload);
void sm_set_callback_rx_json(mesh_rx_json_handler_t rx_json_handler);
void mesh_bcast_string(std::string text);

}/*closing of extern "C" {*/
#endif
