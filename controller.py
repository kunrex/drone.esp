import time, asyncio, array

from pynput import keyboard
from pynput.keyboard import Key, KeyCode

from bleak import BleakClient, BleakScanner

inputs = "abcdef01-1234-5678-1234-56789abcdef0"

def clip(value, max_value, min_value):
    return max(min(value, max_value), min_value)

pressed = dict()
def on_press(key):
    pressed[key] = True

def on_release(key):
    pressed[key] = False

def is_pressed(key):
    if key in pressed.keys():
        return pressed[key]

    return False

w,a,s,d = KeyCode.from_char('w'), KeyCode.from_char('a'), KeyCode.from_char('s'), KeyCode.from_char('d')

print("Use the up and down arrow keys to control base thrust")
print("Use WASD to control the pitch and the roll of the drone. Use the right and left arrow keys to control the yaw")

print()
print("Press escape to end")

async def scan():
    return await BleakScanner.find_device_by_name("david.drone")

async def main():
    def input_data(in_data, index: int, change: float, key_inc: KeyCode | Key, key_dec: KeyCode | Key) -> bool:
        if is_pressed(key_inc):
            in_data[index] += change
            return True
        elif is_pressed(key_dec):
            in_data[index] -= change
            return True
        else:
            return False

    then = time.time()
    data = [0.0 for i in range(0, 4)]

    device = await scan()
    if not device:
        print("Device not found")
        return

    print("device found:", device.address)

    async with BleakClient(device) as client:
        await client.connect()
        await client.get_services()

        print("connected and got services")

        while True:
            if is_pressed(Key.esc):
                break

            now = time.time()
            dt = now - then
            then = now

            angle_dt = dt * 45
            thrust_dt = dt * 200

            flag1, flag2, flag3, flag4 = (input_data(data, 1, angle_dt, w, s),
                                          input_data(data, 2, angle_dt, d, a),
                                          input_data(data, 3, thrust_dt, Key.up, Key.down),
                                          True)

            if is_pressed(Key.right):
                data[0] = 1.0
            elif is_pressed(Key.left):
                data[0] = -1.0
            else:
                data[0] = 0.0
                flag4 = False

            if flag1 | flag2 | flag3 | flag4:
                for i in range(0, 3):
                    data[i] = clip(data[i], 20.0, -20.0)

                data[3] = clip(data[3], 255.0, 0.0)

                await client.write_gatt_char(inputs, array.array('f', data).tobytes(), response=False)
                await asyncio.sleep(0.050)

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    asyncio.run(main())
    listener.join()