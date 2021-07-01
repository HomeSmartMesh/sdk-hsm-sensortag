#include <stdint.h>

//only c++ supported
#ifdef __cplusplus
#include <string>
//https://nlohmann.github.io/json/
#include <json.hpp>
using json = nlohmann::json;
extern "C" {

namespace sm
{
    const uint8_t bcast_header_length = 4;
    const uint8_t p2p_header_length   = 5;
    const uint8_t max_msg_size        = CONFIG_ESB_MAX_PAYLOAD_LENGTH - p2p_header_length;

    namespace control
    {
        const uint8_t broadcast = 0x80;
        const uint8_t message   = 0x40;
        const uint8_t msg_req   = 0x20;
        const uint8_t send_ack  = 0x10;
        const uint8_t msg_needs_ack = 0x70;
        const uint8_t msg_no_ack = 0x60;
    }

    enum struct pid: uint8_t {
        text            =  0x16,
        file_info       =  0x20,//request with ack
        file_sequence   =  0x21,
        file_status     =  0x22//pid only is request, response comes with struct
    };
    namespace file
    {
        typedef struct 
        {
            uint8_t pid;
            uint8_t seq_size;//all but potentially not last
            uint16_t nb_seq;
            uint32_t size;//in bytes
            uint32_t crc;
        }info_t;//complete message, no payload
        typedef struct 
        {
            uint16_t    seq_id;//starting from 0
            uint32_t    offset;
        }sequence_header_t;//rest is payload

        typedef struct 
        {
            uint16_t nb_success;
            uint16_t nb_missing;
        }status_t;//payload is list of uint16_t missing ids
    }

}

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

typedef void (*mesh_rx_json_handler_t)(std::string &topic, json &data);
std::string sm_get_uid();
std::string sm_get_topic();
bool is_self(std::string &payload);
void sm_set_callback_rx_json(mesh_rx_json_handler_t rx_json_handler);
void mesh_bcast_string(std::string text);
void mesh_bcast_json(json &data);

void mesh_send_json(json &data,uint8_t node_id);
void mesh_send_data(sm::pid pid,uint8_t dest,uint8_t * data,uint8_t size);

}/*closing of extern "C" {*/
#endif /*__cplusplus*/
