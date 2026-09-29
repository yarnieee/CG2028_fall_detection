import socket
import os
from twilio.rest import Client


#INIT SERVER
#server selects a port and waits there for a client to connect
server = socket.socket()
server.bind(('127.0.0.1', 12345))
server.listen() #indefinite waiting


#INIT TWILIO
# Loads credentials automatically from environment variables
twilio_phone_number = "+17372508034"
family_phone_number = "6587786779"

time = ""
address = "4 Engineering Drive 3, Singapore 117583"
emergency_contact_string = f"Help! I've fallen at {time} and I can't get up! Call 995 to {address} now!!!"

client = Client(account_sid, auth_token)

message = client.messages.create(
    body=emergency_contact_string,
    from_=twilio_phone_number,  # Your Twilio phone number
    to=family_phone_number      # The recipient's phone number
)

print(f"Message sent successfully! SID: {message.sid}")

#server has found a friend, yippee
#server accepts the client's connection and they move to a new port (addr) together
server_new, addr = server.accept()
print('Connected to: ' + str(addr))

#
# main body, send and receive works exactly the same as the client. see client for send/recv example.
#

#close both
server_new.close()
server.close()

#if you want to allow connection from another laptop, use 0.0.0.0 to allow all connections. 127.0.0.1 accepts only self, and <your IP addr> accepts others on same network, but not self. *firewall instructions not included.