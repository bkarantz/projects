import serial
import threading


# Initialize bluetooth connections
try:
    alarm = serial.Serial("COM12", 9600, timeout = 1)
    light_switch = serial.Serial("COM7", 9600, timeout = 1)
except Exception as e:
    print(f"Bluetooth Error: {e}")

def main():
    while True:
        val = input("Input: ")
        if type(val) != str:
            print("Invalid input")
        else:
            if val.lower() == "on":
                light_on()
            elif val.lower() == "off":
                light_off()
            elif val.lower() == "alarm":
                alarm_on()
            elif val.lower() == "exit":
                print("Shutting Down")
                return
            else:
                print("Invalid input")

def light_on():
    timeout = 5

    def turn_on():
        try:
            light_switch.write(bytes([1]))
            print("Lights successfully turned on")
        except Exception as e:
            print(f"Bluetooth Error: {e}")
            try:
                light_switch.close()
            except:
                pass
    
    thread = threading.Thread(target=turn_on)
    thread.start()
    thread.join(timeout=timeout) # Stops program after 5s if message is not sent

    if thread.is_alive():
        print("Network timeout")
    
    main()

def light_off():
    timeout = 5

    def turn_off():
        try:
            light_switch.write(bytes([0]))
            print("Lights successfully turned off")
        except Exception as e:
            print(f"Bluetooth Error: {e}")
            try:
                light_switch.close()
            except:
                pass
    
    thread = threading.Thread(target=turn_off)
    thread.start()
    thread.join(timeout=timeout) #Stops program after 5s if message is not sent

    if thread.is_alive():
        print("Network timeout")
    
    main()

def alarm_on():
    try:
        alarm.write(bytes([1]))
    except Exception as e:
        print(f"Bluetooth Error: {e}")
        try:
            alarm.close()
        except Exception as e:
            pass
        return
    
    print("Alarm on. Type OFF to turn alarm off")

    while True:
        val = input()
        if type(val) == str and val.lower() == "off":
            try:
                alarm.write(bytes([0]))
            except Exception as e:
                print(f"Bluetooth Error: {e}")
                try:
                    alarm.close()
                except Exception as e:
                    pass
            break
        else:
            print("Invalid input")

    print("Alarm turned off.")

print("Welcome back!\nType ON to turn lights on\nType OFF to turn lights off\nType ALARM to turn alarm on\nType EXIT to exit anytime")
main()
