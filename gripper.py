import asyncio
from viam.rpc.server import Server
from viam.components.gripper import Gripper
import RPi.GPIO as GPIO

close_pin = 13
open_pin = 11
power_pin = 15

class SoftGripper(Gripper):

    def __init__(self, name: str):
        super(SoftGripper,self).__init__(name)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(11, GPIO.OUT)
        GPIO.setup(15, GPIO.OUT)

    async def open(self):
        GPIO.output(11, 0)
        GPIO.output(15, 1)
        GPIO.output(13, 1)

    async def grab(self):
        GPIO.output(13, 0)
        GPIO.output(15, 1)
        GPIO.output(11, 1)

    async def stop(self):
        GPIO.output(11, 0)
        GPIO.output(15, 0)
        GPIO.output(13, 0)

async def main():
    srv = Server(components=[SoftGripper("soft_gripper")])
    await srv.serve()

if __name__ == "__main__":
   asyncio.run(main())
