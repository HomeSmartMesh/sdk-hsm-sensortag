import pylink

def get_uid():
    d = jlink.memory_read8(0x10000060, 8)
    res = "0x%02X %02X %02X %02X %02X %02X %02X %02X"%(d[3],d[2],d[1],d[0],d[7],d[6],d[5],d[4])
    return res

def print_uid_32():
    device_id = jlink.memory_read32(0x100000A4, 2)
    ids1 = "{0:#0{1}X}".format(device_id[0],10)
    ids2 = "{0:#0{1}X}".format(device_id[1],10)
    return (ids1,ids2)

def start():
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect('NRF52840_XXAA', verbose=True)
    print('ARM Id: %d' % jlink.core_id())
    print('CPU Id: %d' % jlink.core_cpu())
    print('Core Name: %s' % jlink.core_name())
    print('Device Family: %d' % jlink.device_family())
    return

jlink = pylink.JLink()
jlink.open()#os.environ['SEG_JLEDU']

start()

uid = get_uid()
print("device uid : %s" % uid )

(ids1,ids2) = print_uid_32()
print("device id : %s %s" % (ids1,ids2) )
