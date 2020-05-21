import io
import socketserver

from threading import Condition
from http import server

IMAGE_WIDTH  = "360"
IMAGE_HEIGHT = "240"

PAGE = """\
<html>
<head/>
<body>
<center><img src="stream.mjpg" width="{IMAGE_WIDTH}" height="{IMAGE_HEIGHT}"></center>
</body>
</html>
"""


class StreamingOutput:

    def __init__(self):
        self.frame     = None
        self.buffer    = io.BytesIO()
        self.condition = Condition()

    def write(self, buffer):
        if buffer.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buffer)


class StreamingHandler(server.BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        pass

    def do_GET(self):

        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with streaming_output.condition:
                        streaming_output.condition.wait()
                        frame = streaming_output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                pass
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):

    allow_reuse_address = True
    daemon_threads      = True


streaming_output = StreamingOutput()
