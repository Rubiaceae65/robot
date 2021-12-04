# Python 3 server example
from http.server import BaseHTTPRequestHandler, HTTPServer
import time
import jinja2
import list_services
import pathlib

hostName = "0.0.0.0"
serverPort = 8002

class MyServer(BaseHTTPRequestHandler):
    def do_GET(self):
        template_dir = pathlib.Path(__file__).parent.resolve()

        jinjaenv = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
        template = jinjaenv.get_template("linkbar.html")

        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(template.render(thelist=list_services.getservices()).encode('utf-8'))

if __name__ == "__main__":        
    webServer = HTTPServer((hostName, serverPort), MyServer)
    print("Server started http://%s:%s" % (hostName, serverPort))

    try:
        webServer.serve_forever()
    except KeyboardInterrupt:
        pass

    webServer.server_close()
    print("Server stopped.")