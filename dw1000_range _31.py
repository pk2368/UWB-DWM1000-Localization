
import sys, time
from ctypes import sizeof, LittleEndianStructure as Structure, Union
from ctypes import c_ubyte as U8, c_short as U16, c_ulonglong as U64
from dw1000_regs import Reg, DW1000, msdelay
from dw1000_spi import Spi

import rospy
from geometry_msgs.msg import Twist

VERSION = "0.16"

# Specify SPI interfaces:
#   "UDP", "<IP_ADDR>", <PORT_NUM>
SPIF1       = "UDP", "192.168.0.171", 1401 
SPIF2       = "UDP", "192.168.0.102", 1401
SPIF3       = "UDP", "192.168.0.225", 1401

# Blink frame with IEEE EUI-64 tag ID
BLINK_FRAME_CTRL = 0xc5
BLINK_MSG=(('framectrl',   U8),
           ('seqnum',      U8),
           ('tagid',       U64))

MSG_FRAME_CTRL = 0xCC41
MSG_HDR = (('framectrl',   U16),
           ('seqnum',      U8),
           ('panid',       U16),
           ('destaddr',    U64),
           ('srceaddr',    U64))

LIGHT_SPEED = 299702547.0
TSTAMP_SEC  = 1.0 / (128 * 499.2e6)
TSTAMP_DIST = LIGHT_SPEED * TSTAMP_SEC

# Class to encapsulate a message frame or header with fixed-length fields
class Frame(object):
    def __init__(self, fields, bytes=[]):
        self.fields = fields
        self.seqnum = 1
        class struct(Structure):
            _pack_   = 1
            _fields_ = fields
        class union(Union):
            _fields_ = [("values", struct), ("bytes", U8*sizeof(struct))]
        self.u = union()
        for n, b in enumerate(bytes[0:sizeof(struct)]):
            self.u.bytes[n] = b
        self.bytes, self.values = self.u.bytes, self.u.values

    # Return frame data
    def data(self):
        self.values.seqnum = self.seqnum
        self.seqnum += 1
        return list(self.bytes)

    # Return string with field values
    def field_values(self, zeros=True):
        flds = [f[0] for f in self.fields]
        return " ".join([("%s:%x" % (f,getattr(self.values, f))) for f in flds])
def control_robot(t3, t4):
    # Create a Twist message object
    move_cmd = Twist()

    # Set linear speed to 0, as we are only concerned with angular speed
    move_cmd.linear.x = 0.05

    # Determine if the robot should turn left or right based on the distance
    if t3 > t4:
        move_cmd.angular.z = 0.3  # Turn left
    else:
        move_cmd.angular.z = -0.3  # Turn right

    # Return the control command
    return move_cmd


# Before the main loop
rospy.init_node("dw1000_control")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

if __name__ == "__main__":

    
    verbose = False
    for arg in sys.argv[1:]:
        if arg.lower() == "-v":
            verbose = True
    spi1 = Spi(SPIF1, '1')
    dw1 = DW1000(spi1)

    spi2 = Spi(SPIF2, '2')
    dw2 = DW1000(spi2)

    spi3 = Spi(SPIF3, '2')
    dw3 = DW1000(spi3)

    if verbose:
        spi1.verbose = spi2.verbose = spi3.verbose = True

    dw1.reset()
    if not dw1.test_irq():
        print("No interrupt from unit 1")
        #sys.exit(1)
    dw2.reset()
    if not dw2.test_irq():
        print("No interrupt from unit 2")
        #sys.exit(1)

    dw3.reset()
    if not dw3.test_irq():
        print("No interrupt from unit 3")
        #sys.exit(1)

    dw1.initialise()
    dw2.initialise()
    dw3.initialise()

    blink1 = Frame(BLINK_MSG)
    blink1.values.framectrl = BLINK_FRAME_CTRL
    blink1.values.tagid = 0x0101010101010101
    blink2 = Frame(BLINK_MSG)
    blink2.values.framectrl = BLINK_FRAME_CTRL
    blink2.values.tagid = 0x0202020202020202
    blink3 = Frame(BLINK_MSG)
    blink3.values.framectrl = BLINK_FRAME_CTRL
    blink3.values.tagid = 0x0303030303030303

    errors = count = 0
    while True:
        # Reset devices if 10 concsecutive errors
        errors += 1
        if errors > 10:
            print("Resetting")
            dw1.softreset()
            dw1.initialise()
            dw2.softreset()
            dw2.initialise()
            dw3.softreset()
            dw3.initialise()
            errors = 0

        # First message
        txdata = blink1.data()
        dw2.start_rx()
        dw3.start_rx()
        dw1.set_txdata(txdata)
        dw1.start_tx()
        rxdata = dw2.get_rxdata()
        rxdata1 = dw3.get_rxdata()
        
        if not rxdata:
            print(dw2.sys_status())
            continue
        dw3.clear_irq()
        
        if not rxdata1:
            print(dw3.sys_status())
            continue
        dw3.clear_irq()

        

        # Second message
        
        txdataB = blink2.data()
        dw1.start_rx()
        dw2.set_txdata(txdataB)
        dw2.start_tx()
        rxdataB = dw1.get_rxdata()
        if not rxdataB:
            print(dw1.sys_status())
            continue
        dw1.clear_irq()
        dtB1 = dw1.rx_time() - dw1.tx_time()
        dtB2 = dw2.tx_time() - dw2.rx_time()
        txB1, rxB1 = dw1.tx_time(), dw2.rx_time()
        txB2, rxB2 = dw2.tx_time(), dw1.rx_time()
        
        
        txdataC = blink3.data()
        dw1.start_rx()
        dw3.set_txdata(txdataC)
        dw3.start_tx()
        rxdataC = dw1.get_rxdata()
        if not rxdataC:
            print(dw1.sys_status())
            continue
        dw1.clear_irq()
        dtC1 = dw1.rx_time() - dw1.tx_time()
        dtC2 = dw3.tx_time() - dw3.rx_time()
        txC1, rxC1 = dw1.tx_time(), dw3.rx_time()
        txC2, rxC2 = dw3.tx_time(), dw1.rx_time()

        # Third message
        txdata = blink1.data()
        dw2.start_rx()
        dw3.start_rx()
        dw1.set_txdata(txdata)
        dw1.start_tx()
        rxdata = dw2.get_rxdata()
        rxdata1 = dw3.get_rxdata()
        if not rxdata:
            print(dw2.sys_status())
            continue
        dw2.clear_irq()

        if not rxdata1:
            print(dw3.sys_status())
            continue
        dw3.clear_irq()

        # Time calculation
        txA3, rxB3, rxC3 = dw1.tx_time(), dw2.rx_time(), dw3.rx_time()
        roundAB = rxB2 - txB1
        roundAC = rxC2 - txC1
        roundB = rxB3 - txB2
        roundC = rxC3 - txC2
        replyB = txB2 - rxB1
        replyC = txC2 - rxC1
        replyAB = txA3 - rxB2
        replyAC = txA3 - rxC2
        
        t1 = ((roundAB * roundB) - (replyB * replyAB)) / (roundAB + roundB + replyB + replyAB)
       

        t2 = ((roundAC * roundC) - (replyC * replyAC)) / (roundAC + roundC + replyC + replyAC)
       
        #print("%7.3f %7.3f" % (t1, t2))
        t3 = t1*TSTAMP_DIST-154.7
        t4 = t2*TSTAMP_DIST-154.3
        if t3>-0.05 and t3<7:
            if t4>-0.05 and t4<7:
                t5=t3
                t6=t4
            else:
                continue
        else:
            continue

        t3_buffer = []
        t4_buffer = []
        buffer_size = 5
        t3_buffer.append(t5)
        t4_buffer.append(t6)

        if len(t3_buffer) > buffer_size:
            t3_buffer.pop(0)
            t4_buffer.pop(0)

        t5_avg = sum(t3_buffer) / len(t3_buffer)
        t6_avg = sum(t4_buffer) / len(t4_buffer)



        print("%7.3f %7.3f" % (t5_avg, t6_avg))
        # t3.append(t2*TSTAMP_DIST-154)
        errors = 0
        # with open("2m_45s_70d.txt", "a") as f:1*TSTAMP_DIST-154s
        #     # Write the current list to the file
        #     f.write(" ".join(str(x) for x in t3) + "\n")
        

        # Print message count
        count += 1
        if count%100 == 0:
            sys.stderr.write(str(count) + ' ')
            sys.stderr.flush()
        # In the main loop, after calculating the distances
        move_cmd = control_robot(t5_avg, t6_avg)
        pub.publish(move_cmd)
        print("robot speed:",(move_cmd))



# EOF1