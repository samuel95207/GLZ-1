import RPi.GPIO as GPIO
import pigpio 

class Servo:
    def __init__(self,PIN=None, PWM_FREQ = 50, mode="SOFTWARE"):
        self.PWM_FREQ = PWM_FREQ
        self.angle = 0

        self.mode = mode
        self.PIN = PIN
        

        if(self.mode == "SOFTWARE"):
            self.pwm = None
        elif(self.mode == "HARDWARE"):
            self.pi = pigpio.pi()

        if(PIN != None):
            self.attach(PIN)

        
    def __exit__(self, exc_type, exc_value, traceback):
        if(self.mode == "SOFTWARE"):
            self.pwm.stop()
            GPIO.cleanup()


    def __del__(self):
        if(self.mode == "SOFTWARE"):
            self.pwm.stop()
            GPIO.cleanup()


    def stop(self):
        if(self.mode == "SOFTWARE"):
            self.pwm.stop()
            GPIO.cleanup()
        elif(self.mode == "HARDWARE"):
            self.pi.set_mode(self.PIN, pigpio.OUTPUT)
            self.pi.write(self.PIN, 0)
            self.pi.stop()
        print("PIN" + str(self.PIN) +" servo stop")




    def attach(self,PIN):
        self.PIN = PIN

        if(self.mode == "SOFTWARE"):
            if(self.pwm):
                self.pwm.stop()

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.PIN, GPIO.OUT)
        
            self.pwm = GPIO.PWM(PIN, self.PWM_FREQ)
            self.pwm.start(0)

        elif(self.mode == "HARDWARE"):
            if(PIN not in (12,13,18,19)):
                print("ERROR: PIN"+PIN+" is not hardware pwm pin")
            self.pi.set_mode(PIN, pigpio.ALT5)

            

        
    def write(self, angle):
        if(angle >= 180 and angle <= 0):
            return 

        self.angle = angle

        

        if(self.mode == "SOFTWARE"):
            dc = (0.05 * self.PWM_FREQ) + (0.19 * self.PWM_FREQ * angle / 180)
            self.pwm.ChangeDutyCycle(dc)
        elif(self.mode == "HARDWARE"):
            dc = int((500 * self.PWM_FREQ + (1900 * self.PWM_FREQ * angle / 180)))
            self.pi.hardware_PWM(self.PIN, self.PWM_FREQ, dc)

         
    def read(self):
        return self.angle