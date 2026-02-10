import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import can
import time

# See https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
class CANSubscriber(Node):

    def __init__(self):
        super().__init__('can_subscriber')
        # Subscriber for CAN requests
        self.subscription = self.create_subscription(
            String,
            'can_msg',
            self.listener_callback,
            10)
        
        # Publishers for canbus data
        # NOTE: This may need to be tuned
        max_queue = 10
        timer_freq = 0.01 # seconds
        self.temp_fet_publisher = self.create_publisher(Float32, 'temp_fet', max_queue)
        self.temp_fet_publisher = self.create_publisher(Float32, 'temp_motor', max_queue)
        self.temp_fet_publisher = self.create_publisher(Float32, 'current_in', max_queue)
        self.temp_fet_publisher = self.create_publisher(Float32, 'pid_position', max_queue)
        self.timer = self.create_timer(timer_freq, self.timer_callback)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        # Parse message for data
        # NOTE: Assuming "vesc_id COMMAND value value_type"
        can_msg = msg.data.split(' ')
        vesc_id = int(can_msg[0])
        command = can_msg[1]
        # Process value
        value_type = can_msg[3]
        if value_type == 'float':
            value = float(can_msg[2])
        elif value_type == 'int':
            value = int(can_msg[2])
        elif value_type == 'string':
            value = can_msg[2]
        else:
            raise TypeError(f"Type {value_type} not known/used.")

        # Build message
        compiled_msg, is_status = build_msg(command=command, value=value, vesc_id=vesc_id)

        # TODO: Add capability to synchronize message consumption
        # Send message
        send_msg(compiled_msg=compiled_msg)

        # TODO if someone needs to manually send status commands, deal with that here

    def timer_callback(self):
        receive_canbus(2)

def receive_canbus(num_messages: int, infty: bool = False):
    """
    Queries the canbus for data. 

    Args:
        num_messages: Number of messages to parse before exiting. 
        infty: Whether to query the canbus until it runs out of messages. 
    """
    channel = 'can0'
    with can.Bus(channel=channel, interface='socketcan') as bus:
        # if you try to query for all messages in the canbus,
        # the canbus publishes more messages than you can parse
        i = 0
        for msg in bus:
            if i == num_messages and not infty:
                break

            # Parse arbitration id
            arbitration_id = bin(msg.arbitration_id)[2:].zfill(29)
            command_id = int(arbitration_id[13:21], 2)
            vesc_id = int(arbitration_id[21:], 2)
            # TODO for now, the values are strangely off by ~1-2, and sometimes 58 instead of 28.
            # Not sure why, might need to investigate electrical. 
            # print(f"Command id {command_id} with vesc id {vesc_id}")

            # Data
            # NOTE this is hacky but it works
            data = bin(int.from_bytes(msg.data, 'big', signed=True))[2:].zfill(64)

            match command_id:
                case 16:
                    # B0-B1: Temp FET in DegC (scale factor 10)
                    temp_fet_bits = data[0:16]
                    temp_fet_raw = int(temp_fet_bits, 2)
                    temp_fet_degc = temp_fet_raw / 10
                    #print(f"Temperature FET: {temp_fet_degc} °C")

                    # B2-B3: Temp Motor in DegC (scale factor 10)
                    temp_motor_bits = data[16:32]
                    temp_motor_raw = int(temp_motor_bits, 2)
                    temp_motor_degc = temp_motor_raw / 10
                    #print(f"Temperature Motor: {temp_motor_degc} °C")

                    # B4-B5: Current In A (scale factor 10)
                    current_bits = data[32:48]
                    current_raw = int(current_bits, 2)
                    current_amps = current_raw / 10
                    #print(f"Current: {current_amps} A")

                    # B6-B7: PID Pos Deg (scale factor 50)
                    pid_pos_bits = data[48:64]
                    pid_pos_raw = int(pid_pos_bits, 2)
                    pid_pos_deg = pid_pos_raw / 50
                    #print(f"PID Position: {pid_pos_deg} degrees")

                # case 28:
                #     # B0-B1: ADC1 in V (scale factor 1000)
                #     adc1_bits = data[0:16]
                #     adc1_raw = int(adc1_bits, 2)
                #     adc1_v = adc1_raw / 1000
                #     print(f"ADC1: {adc1_v} V")
                    
                #     # B2-B3: ADC2 in V (scale factor 1000)
                #     adc2_bits = data[16:32]
                #     adc2_raw = int(adc2_bits, 2)
                #     adc2_v = adc2_raw / 1000
                #     print(f"ADC2: {adc2_v} V")
                    
                #     # B4-B5: ADC3 in V (scale factor 1000)
                #     adc3_bits = data[32:48]
                #     adc3_raw = int(adc3_bits, 2)
                #     adc3_v = adc3_raw / 1000
                #     print(f"ADC3: {adc3_v} V")
                    
                #     # B6-B7: PPM % / 100 (scale factor 1000)
                #     ppm_bits = data[48:64]
                #     ppm_raw = int(ppm_bits, 2)
                #     ppm_percent = ppm_raw / 1000
                #     print(f"PPM: {ppm_percent} %")

            i += 1

def send_msg(compiled_msg: can.message.Message):
    """Immediately send a compiled CAN message"""
    channel = 'can0'
    #print(f"Sending {compiled_msg.arbitration_id} with {compiled_msg.data}")
    with can.Bus(channel=channel, interface='socketcan') as bus:
        bus.send(compiled_msg)

def build_msg(command: str, value: int, vesc_id: int, raw: bool = False):
    """
    Builds a VESC message. Can be sent to VESC with bus.send(msg). 
    Simple commands set a value and are self-explanatory. 
    Status commands provide 2-4 values in response. Value is zeroed for status. 

    Possible commands:

    Single-Frame (simple) Commands
    - CAN_PACKET_SET_DUTY
    - CAN_PACKET_SET_CURRENT
    - CAN_PACKET_SET_CURRENT_BRAKE
    - CAN_PACKET_SET_RPM
    - CAN_PACKET_SET_POS
    - CAN_PACKET_SET_CURRENT_REL
    - CAN_PACKET_SET_CURRENT_BRAKE_REL
    - CAN_PACKET_SET_CURRENT_HANDBRAKE
    - CAN_PACKET_SET_CURRENT_HANDBRAKE_REL

    Status Commands
    - CAN_PACKET_STATUS
        - ERPM
        - Current
        - Duty Cycle
    - CAN_PACKET_STATUS_2
        - Amp Hours
        - Amp Hours Chg
    - CAN_PACKET_STATUS_3
        - Watt Hours
        - Watt Hours Chg
    - CAN_PACKET_STATUS_4
        - Temp FET
        - Temp Motor
        - Current In
        - PID Pos
    - CAN_PACKET_STATUS_5
        - Tachometer
        - Volts In
    - CAN_PACKET_STATUS_6
        - ADC1
        - ADC2
        - ADC3
        - PPM

    Args:
        command: name of command.
        value: value to send motor
        vesc_id: id of vesc unit

    Returns:
        PyCAN message object and whether the command was status (unpacked tuple). 
        Or, if raw == True, a tuple of (id(bitstring), data(bytes))
    """
    command_id = None
    scaling = None
    is_status = False

    # search for command
    match command:
        # Simple commands
        case "CAN_PACKET_SET_DUTY":
            # Unit is % / 100
            # Desc is Duty Cycle
            # Range is -1.0 to 1.0
            command_id = 0
            scaling = 100000
        case "CAN_PACKET_SET_CURRENT":
            # Unit is A
            # Desc is Motor Current
            # Range is -MOTOR_MAX to MOTOR_MAX
            command_id = 1
            scaling = 1000
        case "CAN_PACKET_SET_CURRENT_BRAKE":
            # Unit is A
            # Desc is Braking Current
            # Range is -MOTOR_MAX to MOTOR_MAX
            command_id = 2
            scaling = 1000
        case "CAN_PACKET_SET_RPM":
            # Unit is RPM
            # Desc is RPM
            # Range is -MAX_RPM to MAX_RPM
            command_id = 3
            scaling = 1
        case "CAN_PACKET_SET_POS":
            # Unit is Degrees
            # Range is 0 to 360
            command_id = 4
            scaling = 1000000
        case "CAN_PACKET_SET_CURRENT_REL":
            # Unit is % / 100
            # Range is -1.0 to 1.0
            command_id = 10
            scaling = 100000
        case "CAN_PACKET_SET_CURRENT_BRAKE_REL":
            # Unit is % / 100
            # Range is -1.0 to 1.0
            command_id = 11
            scaling = 100000
        case "CAN_PACKET_SET_CURRENT_HANDBRAKE":
            # Unit is A
            # Range is -MOTOR_MAX to MOTOR_MAX
            command_id = 12
            scaling = 1000
        case "CAN_PACKET_SET_CURRENT_HANDBRAKE_REL":
            # Unit is % / 100
            # Range is -1.0 to 1.0
            command_id = 13
            scaling = 100000

        # Status commands
        case "CAN_PACKET_STATUS":
            # ERPM is RPM
            # Current is A
            # Duty Cycle is % / 100
            command_id = 9
            scaling = 0
            is_status = True
        case "CAN_PACKET_STATUS_2":
            # Amp Hours is Ah
            # Amp Hours Chg is Ah
            command_id = 14
            scaling = 0
            is_status = True
        case "CAN_PACKET_STATUS_3":
            # Watt Hours is Wh
            # Watt Hours Chg is Wh
            command_id = 15
            scaling = 0
            is_status = True
        case "CAN_PACKET_STATUS_4":
            # Temp FET is DegC
            # Temp Motor is DegC
            # Current In is A
            # PID Pos is Deg
            command_id = 16
            scaling = 0
            is_status = True
        case "CAN_PACKET_STATUS_5":
            # Tachometer is EREV
            # Volts In is V
            command_id = 27
            scaling = 0
            is_status = True
        case "CAN_PACKET_STATUS_6":
            # ADC1 is V
            # ADC2 is V
            # ADC3 is V
            # PPM is % / 100
            command_id = 28
            scaling = 0
            is_status = True

        case _:
            raise Exception(f"{command} with value {value} not known.")

    # build arbitration_id
    unused_bits = '0000000000000'
    command_bits = bin(command_id)[2:].zfill(8)
    vesc_bits = bin(vesc_id)[2:].zfill(8)
    # finalize arb_id
    id = unused_bits + command_bits + vesc_bits
    # specify base 2 for type conversion
    int_id = int(id, 2)

    # build data
    int_data = value * scaling
    int_data = int(int_data)
    # VESC uses big endian, and we need 4 bytes
    data = int_data.to_bytes(4, byteorder='big', signed=True)

    # build pycan msg
    if raw:
        return id, data
    # VESC uses extended ids
    return can.Message(arbitration_id=int_id, data=data, is_extended_id=True), is_status

def main(args=None):
    rclpy.init(args=args)

    can_subscriber = CANSubscriber()

    print("Starting CAN subscriber...")
    rclpy.spin(can_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
