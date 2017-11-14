## frAIburg python tornado thrift with web app
=====================
- implementation of a proof of concept for the Web App sending data to a python thrift server

- main function of the web app are:
  add a new person with name and image and select a person in a list to drive to
  the Material Design CSS Framework mui is used: https://github.com/muicss/mui
  js dependencies: thrift, mui Pillow (for scipy.ndimage import imread)

web app Support:
- Android 3.0 browser
- Chrome for Android (0.16)
- Firefox Mobile 10.0
- iOS6 Safari and Chrome (partial support)
test on:
- Chrome for Android Version 62.0
- Chrome ios Version 62.0
- Chrome mac Version 61.0.3163.100
- Safari ios 9.3.1

- Python Tornado to serve Thrift HTTP requests. based on: https://omri.org.il/2012/12/22/python-tornado-apache-thrift-and-javascript/
py server dependencies: thrift and tornado

'$ thrift -r -gen js  -out static/gen-js/ newservice.thrift '
'$ thrift -r -gen py  newservice.thrift '
'$ python server_tornado.py '
then go to http://localhost:8888/static/index.html

Note: js thrift only supports the TJSONProtocol

Project layout:   
1. newservice.thrift - describing the thrift interface, generating files used in newservicehandler and in static/index.html
2. server_tornado.py - Tornado powered Thrift HTTP server - accepts both POST and web socket transport protocols  
3. newservicehandler.py - py implementation to handle client requests
5. static/index.html - HTML client with Java script Thrift library - accessible through http://localhost:8888/static/index.html when running server_tornado.py  

Router configuration:

FRITZ!Box als DSL-Modem nutzen
Alle angeschlossenen Computer bauen Ihre eigene Internetverbindung mit eigener Zugangssoftware auf

fot the demo task only use wifi!

### nginx
- use the absolute path
- run test:
$ sudo nginx -t -c /Users/markusmerklinger/github/frAIburg_audicup/python/frAIburg/thrift_web_app_prototyp/nginx.conf

- run test:
sudo nginx  -c /Users/markusmerklinger/github/frAIburg_audicup/python/frAIburg/thrift_web_app_prototyp/nginx.conf
- stop nginx server :
sudo nginx -s stop

router: use hairpin NAT

OS X:
dig, nslookup, and host will not see entries in it because they bypass the system's resolver and do raw DNS lookups
check with $ dscacheutil -q host -a name fraiburg.io
