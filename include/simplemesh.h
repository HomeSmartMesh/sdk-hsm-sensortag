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
    const uint32_t max_file_size      = 4096;

    namespace control
    {
        const uint8_t broadcast = 0x80;
        const uint8_t ack       = 0x40;
        const uint8_t msg_req   = 0x20;
        const uint8_t send_ack  = 0x10;
        const uint8_t msg_needs_ack = 0x70;
        const uint8_t msg_no_ack = 0x60;
    }

    enum struct pid: uint8_t {
        ping            =  0x00,
        node_id_get     =  0x01,//(1) ['uid': 8 byets in 16 chars text]
        node_id_set     =  0x02,//(2) ['uid:shortid' : short id 1 byte in 2 chars text]
        node_id_reset   =  0x03,//(3) on new coordinator all nodes shall request a new short id
        text            =  0x16,//(22)
        json            =  0x17,//(23)
        file_info       =  0x20,
        file_section    =  0x21
    };
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

typedef struct
{
	std::string name;
	uint32_t    size;
	uint8_t     seq_size;
	uint8_t     seq_count;
	uint32_t    nb_seq;
	uint8_t     last_seq_size;
    uint8_t*    buffer_p;
    uint8_t     buffer[sm::max_file_size];
}file_t;

typedef void (*mesh_rx_handler_t)(message_t*);

void sm_start();

//------------------------- CPP wrapper interfaces -------------------------

typedef void (*mesh_rx_json_handler_t)(uint8_t source, std::string &topic, json &data);
uint8_t sm_get_sid();
std::string sm_get_uid();
std::string sm_get_topic();
std::string sm_get_base_topic();

bool is_broadcast(std::string &payload);
bool is_self(std::string &payload);
void sm_set_callback_rx_json(mesh_rx_json_handler_t rx_json_handler);

void mesh_bcast_packet(sm::pid pid,uint8_t * data,uint8_t size);
void mesh_bcast_json(json &data);
void mesh_bcast_json_to(json &data,std::string &target);

void mesh_send_json(uint8_t dest_id, json &data);
void mesh_send_file(const char * name, uint8_t dest,uint8_t* data, uint32_t size);

#ifdef CONFIG_SM_GPIO_DEBUG
    void sm_gpio_init(const struct device *gpio_dev);
#endif

int64_t sm_rx_sync_ms(int lseq,int64_t delay);
int64_t sm_sync_ms(int lseq,int64_t start,int64_t delay);

void sm_diag(json &data);

void sm_start_rx();
void sm_stop_rx();

}/*closing of extern "C" {*/
#endif /*__cplusplus*/
