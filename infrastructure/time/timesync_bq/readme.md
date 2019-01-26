#Timesync

##Master
The Master BQ publishes an empty Message - the header of the message will contain the timestamp and sequence number.

##Client
The client BQ reads the master channel and upon reciept of a message from the master, publishes a ClientTimesyncMessage which contains the following:
* Master Timestamp
* Master Sequence Number
* Hostname of the machine running the Client
The header of this ClientTimesyncMessage contains the client's timestamp and sequence number.
The client also prints out the difference in timestamps between Master and Client (client_timestamp - master_timestamp)

##Usage
The Master BQ should be run on the user PC. The Client BQ should be run on all other machines. The client timesync channel will contain time information about both the Master and the Clients allowing us to reconstruct the time offset between the two machines.