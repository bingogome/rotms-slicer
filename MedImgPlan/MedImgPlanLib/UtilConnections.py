#
# Connection
#

import socket
import json
import slicer

class UtilConnections():
  """
  Connection class.
  Blocking send and receive
  """

  def __init__(self, configPath, modulesufx):

    # port init
    with open(configPath+"Config.json") as f:
      configData = json.load(f)
    
    self._sock_ip_receive = configData["IP_RECEIVE_"+modulesufx]
    self._sock_ip_send = configData["IP_SEND_"+modulesufx]
    self._sock_receive_port = configData["PORT_RECEIVE_"+modulesufx]
    self._sock_send_port = configData["PORT_SEND_"+modulesufx]

    self._sock_receive = None
    self._sock_send = None
  
  def setup(self):
    self._sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._sock_receive.bind((self._sock_ip_receive, self._sock_receive_port))
    self._sock_receive.settimeout(0.5)
    
  def clear(self):
    if self._sock_receive:
      self._sock_receive.close()
    if self._sock_send:
      self._sock_send.close()
      
  def utilSendCommand(self, msg, errorMsg="Failed to send command ", res=False):
    if len(msg) > 150:
      raise RuntimeError("Command contains too many characters.")
    try:
      self._sock_send.sendto( \
        msg.encode('UTF-8'), (self._sock_ip_send, self._sock_send_port) )
      try:
        data = self._sock_receive.recvfrom(512)
      except socket.error:
        raise RuntimeError("Command response timedout")
    except Exception as e:
      slicer.util.errorDisplay(errorMsg+str(e))
      import traceback
      traceback.print_exc()
    if res:
      return data
