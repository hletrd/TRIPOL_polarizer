from flask import Flask, render_template, send_from_directory
import serial
import serial.tools.list_ports
import threading

app = Flask(__name__)

def run_server():
	app.run(host=bind_ip, debug=True, port=bind_port)

@app.route('/')
def index():
	return render_template('_basic.html', ports=serialhandler.get_port_list())

@app.route('/get/angle/now')
def get_angle():
	return str(serialhandler.angle_now)

@app.route('/get/angle/to')
def get_angle_to():
	return str(serialhandler.angle_to)

@app.route('/open/<path:port>')
def open_serial(port):
	serialhandler.connect(port[1:])
	return '1'

@app.route('/move/<string:angle>')
def move_angle(angle):
	if (360 >= float(angle) >= 0):
		serialhandler.move_angle(str(float(angle)))
		return '1'
	return '0'

@app.route('/static/<path:path>')
def send_static(path):
	return send_from_directory('static', path, as_attachment=False)



class SerialHandler(object):
	def __init__(self):
		self.Serial = serial.Serial()
		self.Serial.baudrate = 115200
		self.Serial.timeout = 0.1
		self.angle_now = 0.0
		self.angle_to = '0.0'
		self.q = ''

	def get_port_list(self):
		result = serial.tools.list_ports.comports()
		return result

	def connect(self, port):
		self.Serial.port = port
		self.Serial.open()
		threading.Timer(0.2, self.read_serial).start()

	def move_angle(self, angle):
		self.Serial.write(angle.encode('utf-8'))
		self.angle_to = angle

	def read_serial(self):
		threading.Timer(0.2, self.read_serial).start()
		try:
			while self.Serial.in_wating > 0:
				self.q += self.Serial.read().decode('utf-8')
		except:
			while self.Serial.inWaiting() > 0:
				self.q += self.Serial.read(1).decode('utf-8')
		splitted = self.q.split('\n\n')
		last = splitted[len(splitted)-1]
		if 'angpos:' in last and 'speed:' in last:
			self.q = ''
			self.angle_now = (float) (last.split('angpos:')[1].split('\n')[0])




if __name__ == '__main__':
	bind_ip = '127.0.0.1'
	bind_port = 8000
	serialhandler = SerialHandler()
	run_server()