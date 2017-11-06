/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */
#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"

//define the type Neighbor with the Node ID and number of pings to it
typedef nx_struct Neighbor {
   nx_uint16_t Node;
   nx_uint16_t pingNumber;
}Neighbor;

typedef nx_struct LinkState {
   nx_uint16_t Dest;
   nx_uint16_t Cost;
   nx_uint16_t Next;
   nx_uint16_t Seq;
   //nx_uint16_t from;
   nx_uint8_t Neighbors[64];
   nx_uint16_t NeighborsLength;
}LinkState;

module Node{
   uses interface Boot;
   
   //list of packets to determine if a node has seen or sent this packet before
   uses interface List<pack> as MarkedPackets;
   //timer to use to fire packets
   uses interface Timer<TMilli> as PeriodicTimer;
   //random number used for timer to make sure it's spaced
   uses interface Random as Random;
   //list of neighboring nodes as seen by current node
   uses interface List<Neighbor> as Neighbors;
   //list of removed nodes from the neighbor list
   uses interface List<Neighbor> as NeighborsDropped;
   //list of nodes and their costs 
   uses interface List<Neighbor> as NeighborCosts;
   //routing table to be used by nodes
   uses interface List<LinkState> as RoutingTable;
   //confirmed table for algo
   uses interface List<LinkState> as Confirmed;
   //tentative table for algo
   uses interface List<LinkState> as Tentative;
   uses interface List<LinkState> as Temp;
   uses interface SplitControl as AMControl;
   uses interface Receive;

   uses interface SimpleSend as Sender;

   uses interface CommandHandler;

}

implementation{
   pack sendPackage;
   uint16_t seqCounter = 0;
   uint16_t accessCounter = 0;
   uint32_t difference = 0;
   uint16_t algopush = 0;
   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
   //puts a packet into the list at the top
   void pushPack(pack Package);
   //finds a packet in the list
   bool findPack(pack *Package);
   //access neighbor list
   void accessNeighbors();
   //starts to flood the LSP packet
   void floodLSP();
   //runs dijkstra's algorithm for shortest path
   //void algorithm();
   void findNext();
   void algorithm(uint16_t Dest, uint16_t Cost, uint16_t Next, uint8_t * Nbors, uint16_t Length);

   void printLSP();

   event void Boot.booted(){
      uint32_t initial;
      uint32_t change;
      call AMControl.start();
      dbg(GENERAL_CHANNEL, "Booted\n");
      //create an initial time to start firing from between 0-999 milliseconds
      initial = call Random.rand32() % 1000;
      //change is the interval to fire between each fire from 10000-25000 milliseconds
      change = 10000 + (call Random.rand32() % 15000);
      //start the timer
      call PeriodicTimer.startPeriodicAt(initial, change);
      //dbg(GENERAL_CHANNEL, "Timer started at %d, firing interval %d\n", initial, change);
   }

   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
      }else{
         //Retry until successful
         call AMControl.start();
      }
   }

   event void PeriodicTimer.fired() {
	accessNeighbors();
	if (accessCounter > 1 && accessCounter % 5 == 0 && accessCounter < 16){
		floodLSP();
		//algorithm(TOS_NODE_ID, 0, 0, 0, call Neighbors.size());
		//printLSP();
		//findNext();
	}
	if (accessCounter > 1 && accessCounter % 20 == 0 && accessCounter < 61)
		algorithm(TOS_NODE_ID, 0, 0);
   }


   event void AMControl.stopDone(error_t err){}

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
		//dbg(FLOODING_CHANNEL, "Packet Received\n");
		if(len==sizeof(pack))
		{
			LinkState DESTI, DEST;
			uint16_t NEXT,SEND,x,y;
			//creates a message with the payload, or message, of the recieved packet
			pack* myMsg=(pack*) payload;
			//check to see if this packet needs to be dropped, either through checking to see if the TTL expired, or if it was listed in the list of sent or seen packets
			if((myMsg->TTL <= 0) || findPack(myMsg)){
				//drops packet by doing nothing and not sending out a new packet
			}
			//else, check to see if the packet is checking for neighbors, deal with it seperately
			else if(myMsg->dest == AM_BROADCAST_ADDR) {
				bool found;
				bool match;
				uint16_t length;
				uint16_t i = 0;
				Neighbor Neighbor1,Neighbor2,NeighborCheck;
				//if the packet is sent to ping for neighbors
				if (myMsg->protocol == PROTOCOL_PING){
					//send a packet that expects replies for neighbors
					//dbg(NEIGHBOR_CHANNEL, "Packet sent from %d to check for neighbors\n", myMsg->src);
					makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, myMsg->TTL-1, PROTOCOL_PINGREPLY, myMsg->seq, (uint8_t *) myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
					pushPack(sendPackage);
					call Sender.send(sendPackage, myMsg->src);
				}
				//if the packet is sent to ping for replies
				else if (myMsg->protocol == PROTOCOL_PINGREPLY){ 
					//update ping number, search and see if the neighbor was found
					//dbg(NEIGHBOR_CHANNEL, "Packet recieved from %d, replying\n", myMsg->src);
					length = call Neighbors.size();
					found = FALSE;
					for (i = 0; i < length; i++){
						Neighbor2 = call Neighbors.get(i);
						//dbg(GENERAL_CHANNEL, "Pings at %d = %d\n", Neighbor2.Node, Neighbor2.pingNumber);
						if (Neighbor2.Node == myMsg->src) {
							//dbg(NEIGHBOR_CHANNEL, "Node found, adding %d to list\n", myMsg->src);
							//reset the ping number if found to keep it from being dropped
							Neighbor2.pingNumber = 0;
							found = TRUE;
						}
					}
				}
				//if the packet is sent to find other nodes
				else if (myMsg->protocol == PROTOCOL_LINKSTATE) {
					//store the LSP in a list of structs
					LinkState LSP;
					LinkState temp;
					Neighbor Ntemp;
					bool end, from, good, found;
					uint16_t j,size,k;
					uint16_t count;
					uint16_t* arr;
					bool same;
					bool replace; 
					count = 0;
					end = TRUE;
					from = FALSE;
					good = TRUE;
					found = FALSE;
					i = 0;
					if (myMsg->src != TOS_NODE_ID){
						arr = myMsg->payload;
						size = call RoutingTable.size();
						LSP.Dest = myMsg->src;
						LSP.Seq = myMsg->seq;
						LSP.Cost = MAX_TTL - myMsg->TTL;
						//dbg(GENERAL_CHANNEL, "myMsg->TTL is %d, LSP.Cost is %d, good is %d\n", myMsg->TTL, LSP.Cost, good);

						if (!call RoutingTable.isEmpty())
						{
							i=0;
							while(!call RoutingTable.isEmpty())
							{
								temp = call RoutingTable.front();
								if((temp.Dest == LSP.Dest) && (LSP.Seq >= temp.Seq) && (LSP.Cost < temp.Cost))
								{
									call RoutingTable.popfront();
								}
								else
								{
									call Temp.pushfront(call RoutingTable.front());
									call RoutingTable.popfront();
								}
							}
							while(!call Temp.isEmpty())
							{
								call RoutingTable.pushfront(call Temp.front());
								call Temp.popfront();
							}
						}
						i=0;
						count=0;
						while(arr[i] > 0)
						{
							LSP.Neighbors[i] = arr[i];
							count++;
							i++;
						}
						LSP.Next = 0;
						LSP.NeighborsLength = count;
						call RoutingTable.pushfront(LSP);
						//printLSP();
						seqCounter++;
						makePack(&sendPackage, myMsg->src, AM_BROADCAST_ADDR, myMsg->TTL-1, PROTOCOL_LINKSTATE, seqCounter, (uint8_t *)myMsg->payload, (uint8_t) sizeof(myMsg->payload));
						pushPack(sendPackage);
						call Sender.send(sendPackage, AM_BROADCAST_ADDR);
					}
				}
				//if we didn't find a match
				if (!found && myMsg->protocol != PROTOCOL_LINKSTATE)
				{
					//add it to the list, using the memory of a previous dropped node
					Neighbor1 = call NeighborsDropped.get(0);
					//check to see if already in list
					length = call Neighbors.size();
					for (i = 0; i < length; i++)
					{
						NeighborCheck = call Neighbors.get(i);
						if (myMsg->src == NeighborCheck.Node)
						{
							match = TRUE;
						}
					}
					if (match == TRUE)
					{
						//already in the list, no need to repeat
					}
					else
					{
						//not in list, so we're going to add it
						//dbg(NEIGHBOR_CHANNEL, "%d not found, put in list\n", myMsg->src);
						LinkState temp;
						Neighbor1.Node = myMsg->src;
						Neighbor1.pingNumber = 0;
						call Neighbors.pushback(Neighbor1);
						
					}
				}
			} 
 			//else, check to see if the packet reached it's destination and see what the purpose/protocal of the packet was 
			else if((myMsg->dest == TOS_NODE_ID) && myMsg->protocol == PROTOCOL_PING) {
				//uint16_t NEXT,x;
				//LinkState DEST;
				NEXT = 0;
				//when protocal = PROTOCAL_PING, the packet was sent as a ping, and not a reply
				dbg(FLOODING_CHANNEL, "Packet is at destination! Package Payload: %s, Sending PING_REPLY to %d\n", myMsg->payload, myMsg->src);
				//make another packet that's the reply from the ping
				makePack(&sendPackage, TOS_NODE_ID, myMsg->src, MAX_TTL, PROTOCOL_PINGREPLY, seqCounter, (uint8_t *) myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
				//increase the sequence number of the next packet that will be sent
				seqCounter++;
				//put the packet into the list
				pushPack(sendPackage);
				
				for(x = 0; x < call Confirmed.size(); x++)
				{
					DEST = call Confirmed.get(x);
					if(myMsg->src == DEST.Dest)
					{
						NEXT == DEST.Next;
					}
				}
				if(NEXT == 0)
				{
					//NEXT = AM_BROADCAST_ADDR;
				}
				//send the new packet
				dbg(ROUTING_CHANNEL, "meant for %d, sending to %d\n", myMsg->src, NEXT);
				call Sender.send(sendPackage, NEXT);
			}
			else if((myMsg->dest == TOS_NODE_ID) && myMsg->protocol == PROTOCOL_PINGREPLY) {
				//the packet is at the right destination, and it is simply a reply, we can stop sending the packet here
				dbg(FLOODING_CHANNEL, "Recieved a reply it was delivered from %d!\n", myMsg->src);
			}
			else {
				//uint16_t y,SEND;
				//LinkState DESTI;
				SEND = 0;
				//all else, wrong destination, flood the packet
				makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1, myMsg->protocol, myMsg->seq, (uint8_t *)myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
				pushPack(sendPackage);
				for(y=0; y < call Confirmed.size(); y++)
				{
					DESTI = call Confirmed.get(y);
					if(myMsg->dest == DESTI.Dest)
					{
						SEND = DESTI.Next;
					}
				}
				if(SEND == 0)
				{
					//SEND = AM_BROADCAST_ADDR;
				}
				dbg(ROUTING_CHANNEL, "meant for %d, sending to %d\n", myMsg->dest, SEND);
				call Sender.send(sendPackage, SEND);
			}
			return msg;
		}
		else
		{
		//all else, we dunno what the packet was to do
		dbg(GENERAL_CHANNEL, "Unknown Packet Use, Error with: %d\n", len);
		return msg;
		}
	}


   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
	uint16_t i,next;
	LinkState temp;
	next = 0;
	for(i = 0; i < call Confirmed.size(); i++)
	{
		temp = call Confirmed.get(i);
		if (temp.Dest == destination)
		{
			next = temp.Next;
		}
	}
	if(next == 0)
	{
		//next = AM_BROADCAST_ADDR;
	}
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      makePack(&sendPackage, TOS_NODE_ID, destination, MAX_TTL, 0, 0, payload, PACKET_MAX_PAYLOAD_SIZE);
      call Sender.send(sendPackage, next);
   }

   event void CommandHandler.printNeighbors(){
	uint16_t i = 0;
	uint16_t length = call Neighbors.size();
	Neighbor beingPrinted;
	if (length == 0){
		dbg(NEIGHBOR_CHANNEL, "No neighbors exist\n");	
	}
	else {
		for (i = 0; i < length; i++){
			beingPrinted = call Neighbors.get(i);
			dbg(NEIGHBOR_CHANNEL, "Neighbor at %d\n", beingPrinted.Node, i);
			}
	}
   }

   event void CommandHandler.printRouteTable(){
	uint16_t k = 0;
	LinkState temp;
	dbg(ROUTING_CHANNEL, "Table for %d\n", TOS_NODE_ID);
	for (k = 0; k < call Confirmed.size(); k++) {
		temp = call Confirmed.get(k);
		dbg(ROUTING_CHANNEL, "Dest: %d, Cost: %d, Next: %d\n", temp.Dest, temp.Cost, temp.Next);
	}
   }

   event void CommandHandler.printLinkState(){}

   event void CommandHandler.printDistanceVector(){}

   event void CommandHandler.setTestServer(){}

   event void CommandHandler.setTestClient(){}

   event void CommandHandler.setAppServer(){}

   event void CommandHandler.setAppClient(){}

   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      memcpy(Package->payload, payload, length);
   }

   void printLSP()
   {
      LinkState temp;
      uint16_t i, j;
      for(i=0; i < call RoutingTable.size(); i++)
		{
			temp = call RoutingTable.get(i);
			dbg(GENERAL_CHANNEL, "LSP from %d, Cost: %d, Next: %d, Seq: %d, Count; %d\n", temp.Dest, temp.Cost, temp.Next, temp.Seq, temp.NeighborsLength);
			for(j=0; j<temp.NeighborsLength; j++)
			{
				//dbg(GENERAL_CHANNEL, "Neighbor at %d\n", temp.Neighbors[j]);
			}
		}
		dbg(GENERAL_CHANNEL, "size is %d\n", call RoutingTable.size());
	}




   void accessNeighbors() {
	//make a packet to send to check neighbors
	pack Pack;
	//test message to be sent
	char* message;
	//increase the access counter
	accessCounter++;
	//dbg(NEIGHBOR_CHANNEL, "Neighbors accessed, %d is checking.\n", TOS_NODE_ID);
	//check to see if neighbors have been found at all
	if (!(call Neighbors.isEmpty())) {
		uint16_t length = call Neighbors.size();
		uint16_t pings = 0;
		Neighbor NeighborNode;
		uint16_t i = 0;
		Neighbor temp; 
		//increase the number of pings in the neighbors in the list. if the ping number is greater than 3, drop the neighbor
		for (i = 0; i < length; i++){
			temp = call Neighbors.get(i);
			temp.pingNumber = temp.pingNumber + 1;
			pings = temp.pingNumber;
			//dbg(ROUTING_CHANNEL, "Pings at %d: %d\n", temp.Node, pings);
			if (pings > 3){
				NeighborNode = call Neighbors.removeFromList(i);
				dbg(NEIGHBOR_CHANNEL, "Node %d dropped due to more than 3 pings\n", NeighborNode.Node);
				call NeighborsDropped.pushfront(NeighborNode);
				i--;
				length--;
			}
		}					
	}
	//ping the list of Neighbors
	message = "pinged neighbors!\n";
	makePack(&Pack, TOS_NODE_ID, AM_BROADCAST_ADDR, 2, PROTOCOL_PING, 1, (uint8_t*) message, (uint8_t) sizeof(message));
	//add the packet to the packet list
	pushPack(Pack);
	//send the packet
	call Sender.send(Pack, AM_BROADCAST_ADDR);
   }

	void pushPack(pack Package) {
		//check to see if the list is full; if so, remove the oldest node
		if (call MarkedPackets.isFull()) {
			call MarkedPackets.popfront();
		}
		call MarkedPackets.pushback(Package);
	}

	bool findPack(pack *Package) {
		uint16_t size = call MarkedPackets.size();
		uint16_t i = 0;
		pack match;
		for (i = 0; i < size; i++) {
			match = call MarkedPackets.get(i);
			if((match.src == Package->src) && (match.dest == Package->dest) && (match.seq == Package->seq)) {
				return TRUE;
			}
		}
		return FALSE;
	}
	
	void floodLSP(){
		//run to flood LSPs, sending info of this node's direct neighbors
		pack LSP;
		LinkState O;
		//dbg(ROUTING_CHANNEL, "LSP Initial Flood from %d\n", TOS_NODE_ID);
		//check to see if there are neighbors to at all
		if (!call Neighbors.isEmpty()){
			uint16_t i = 0;
			uint16_t length = call Neighbors.size();
			uint16_t directNeighbors[length+1];
			Neighbor temp;
			//dbg(ROUTING_CHANNEL, "length = %d/n", length);

			//move the neighbors into the array
			for (i = 0; i < length; i++) {
				temp = call Neighbors.get(i);
				directNeighbors[i] = temp.Node;
			}
			//set a negative number to tell future loops to stop!
			directNeighbors[length] = 0;
			//directNeighbors[length+1] = TOS_NODE_ID;
			//dbg(ROUTING_CHANNEL, "this should be 0: %d\n", directNeighbors[length]);
			//start flooding the packet
			makePack(&LSP, TOS_NODE_ID, AM_BROADCAST_ADDR, MAX_TTL-1, PROTOCOL_LINKSTATE, seqCounter++, (uint16_t*)directNeighbors, (uint16_t) sizeof(directNeighbors));
			pushPack(LSP);
			call Sender.send(LSP, AM_BROADCAST_ADDR);
		}
	}

	void removefunction(uint16_t i)
	{
		LinkState temp, temp2;
		temp2 = call Tentative.get(i);
		while(!call Tentative.isEmpty())
		{
			temp = call Tentative.front();
			if(temp.Dest == temp2.Dest)
			{
				call Tentative.popfront();
			}
			else
			{
				call Temp.pushfront(call Tentative.front());
				call Tentative.popfront();
			}
		}
		while(!call Temp.isEmpty())
		{
			call Tentative.pushback(call Temp.front());
			call Temp.popfront();
		}
	}
		
		
	void algorithm(uint16_t Dest, uint16_t Cost, uint16_t Next) {
		LinkState Link, temp, temp2, temp3, temp4;
		Neighbor next;
		uint8_t i, j, k, m, n, p, q, r;
		bool inTent, inCon;
		Link.Dest = Dest;
		Link.Cost = Cost;
		Link.Next = Next;
		j = 0;
		call Confirmed.pushfront(Link);
		
		if (Link.Dest != TOS_NODE_ID) {
			for (i = 0; i < call RoutingTable.size(); i++) {
				temp = call RoutingTable.get(i);
				if (temp.Dest == Dest) {
					for (j = 0; j < temp.NeighborsLength; j++) {
						if (Dest.Neighbors[j] > 0) {
							inTent = FALSE;
							inCon = FALSE;
							if (!call Tentative.isEmpty()) {
								for (k = 0; k < Tentative.size(); k++) {
									temp2 = call Tentative.get(k);
									if (temp2.Dest == temp.Neighbors[j]) {
										inTent = TRUE;
									}
								}
							}
							if (!call Confirmed.isEmpty()) {
								for (m = 0; m < call Confirmed.size(); m++) {
									temp3 = call Confirmed.get(m);
									if (temp3.Dest == temp.Neighbors[j]) {
										inCon = TRUE;
									}
								}
							}
							if (!inTent && !inCon) {
								temp.Dest = temp.Neighbors[j];
								temp.Cost = Cost+1;
								temp.Next = Dest;
								call Tentative.pushfront(temp);
							}
						}
					}
				}
			}
		}
		
		else {
			for (j = 0; j < call Neighbors.size(); j++) {
				next = call Neighbors.get(j);
				for (n = 0; n < call RoutingTable.size(); n++) {
					temp = call RoutingTable.get(n);
					if (temp.Dest == next.Dest) {
						inTent = FALSE;
						inCon = FALSE;
						if (!call Tentative.isEmpty()) {
							for (k = 0; k < call Tentative.size(); k++) {
								temp2 = call Tentative.get(k);
								if (temp2.Dest == temp.Dest && temp2.Cost >= temp.Cost) {
									temp4 = call Tentative.removefromList(k);
									temp4.Cost = temp.Cost;
									temp4.Next = temp.Next;
									call Tentative.pushfront(temp4);
									inTent = TRUE;
								}
							}
						}
						if (!call Confirmed.isEmpty()) {
							for (m = 0; m < call Confirmed.size(); m++) {
								temp3 = call Confirmed.get(m);
								if (temp3.Dest == temp.Neighbors[j]) {
									inCon = TRUE;
								}
							}
						}
						if (!inTent && !inCon) {
							call Tentative.pushFront(temp);
						}
					}
				}
			}
		}
		
		inCon = FALSE;
		p = call Tentative.size();
		
		if (p == 1) {
			temp4 = call Tentative.get(0);
			call Tentative.popback();
			for (m = 0; m < call Confirmed.size(); m++) {
				temp3 = call Confirmed.get(m);
				if (temp4.Dest == temp3.Dest) {
					inCon = TRUE;
				}
				if (!inCon) {
					if (temp4.Next == temp3.Dest) }
						temp4.Next = temp3.Next;
						for (j = 0; j < call Neighbors.size(); j++) {
							next = call Neighbors.get(j);
							if (temp4.Next == next.Dest) {
								break;
							}
						}
					}
				}
			}
			if (!inCon) {
				algorithm(temp4.Dest, temp4.Cost, temp4.Next);
			}
		}
		
		if (p > 1) {
			inCon = FALSE;
			temp4 = call Tentative.get(0);
			q = 0;
			for (k = 1; k < call Tentative.size(); k++) {
				temp3 = call Tentative.get(k);
				if (temp4.Cost > temp3.Cost) {
					q = k;
					temp4 = temp3;
				}
				else if (temp4.Cost == temp3.Cost && temp4.Dest > temp3.Dest) {
					q = k;
					temp4 = temp3;
				}
			}
			temp2 = call Tentative.get(q);
			for (m = 0; m < call Confirmed.size(); m++) {
				temp3 = call Confirmed.get(q);
				if (temp2.Dest == temp3.dest) {
					inCon = TRUE;
				}
				if (!inCon) {
					if (temp4.Next == temp3.Dest) {
						temp4.Next = temp3.Next;
						for (j = 0; j < call Neighbors.size(); j++) {
							next = call Neighbors.get(j);
							if (temp4.Next == next.Dest) {
								break;
							}
						}
					}
				}
			}
		}
		
		if (!inCon) {
			if (call Tentative.size() - 1 > 1 && q == call Tentative.size() -1) {
				call Tentative.popback();
			}
			else {
				call Tentative.removeFromList(q);
				call Tentative.popback();
			}
			algorithm(temp4.Dest, temp4.Cost, temp4.Next);
		}
		
	}

	// void algorithm(uint16_t Dest, uint16_t Cost, uint16_t Next, uint8_t* Nbors, uint16_t Length) {
	//	LinkState temp;
    //	Neighbor temp2;
	//	Neighbor TEMP;
	//	LinkState temp3;
		//LinkState temp4;
		//LinkState temp5;
		//LinkState temp6;
		//LinkState temp7;
		//LinkState minTemp;
		//uint16_t i,j,k,l,m,minCost,minInt;
		//uint16_t tentInt;
		//uint8_t NeighborsArr[64];
		//bool inTentList;
		//bool inConList;
		//dbg(GENERAL_CHANNEL, "we made it!\n");
		//temp.Dest = Dest;
		//temp.Cost = Cost;
		//temp.Next = Next;
		//dbg(ROUTING_CHANNEL, "here?1\n");
		//temp.NeighborsLength = Length;
		//dbg(ROUTING_CHANNEL, "here?2\n");
		//dbg(ROUTING_CHANNEL, "temp.Dest = %d, temp.Cost = %d, temp.Next = %d, temp.Length = %d\n", Dest, Cost, Next, Length);
		//if(algopush == 0 || temp.Dest != TOS_NODE_ID)
		//{
		//	call Confirmed.pushfront(temp);
		//	algopush++;
		//}
	//	if (temp.Dest != TOS_NODE_ID) {
		//	//dbg(ROUTING_CHANNEL, "bleh %d\n", 1);
		//	for(i=0; i<call RoutingTable.size(); i++)
		//	{
			//dbg(ROUTING_CHANNEL, "bleh %d\n", 2);
		//		temp6 = call RoutingTable.get(i);
	//			if(temp6.Dest == Dest)
	//			{
	//				//dbg(ROUTING_CHANNEL, "bleh %d\n", 3);
		//			break;
		//		}
		//	}
	//		for (i = 0; i < temp6.NeighborsLength; i++) {
				//dbg(ROUTING_CHANNEL, "bleh %d, NeighborsLength: %d\n", 4, temp6.NeighborsLength);
	//			NeighborsArr[i] = temp6.Neighbors[i];
				//NeighborsArr = Nbors;
	//		}
	//		temp = temp6; 
	//	}
		//else
	//	{
		//	for(i = 0; i < call Neighbors.size(); i++)
		//	{
	//			TEMP = call Neighbors.get(i);
		//		NeighborsArr[i] = TEMP.Node;
	//		}
		//}
		//for (j = 0; j < call Neighbors.size(); j++){
		//	temp2 = call Neighbors.get(j);
		//	for (i = 0; i < call RoutingTable.size(); i++){
			//	temp3 = call RoutingTable.get(i);
			//	inTentList = FALSE;
			//	inConList = FALSE;
			//	if (temp.Dest == TOS_NODE_ID) {
			//		if(temp2.Node == temp3.Dest)
			//		{
			//			temp3.Next = temp2.Node;
			//			if (!call Tentative.isEmpty()) {
			//				for (k = 0; k < call Tentative.size(); k++){
			//					temp4 = call Tentative.get(k);
			//					if (temp4.Dest == temp3.Dest) {
			//						inTentList = TRUE;
			//						tentInt = k;
			//					}
			//				}
			//			}
			//			if (!call Confirmed.isEmpty()) {
			//				for (k = 0; k < call Confirmed.size(); k++){
			//					temp4 = call Confirmed.get(k);
		//						if (temp4.Dest == temp3.Dest) {
			//						inConList = TRUE;
			//					}
		//					}
		//				}
		//				if (!inTentList && !inConList) {
		//					call Tentative.pushfront(temp3);
			//			}
		//				else if (inTentList) {
			//				temp4 = call Tentative.get(tentInt);
			//				if (temp3.Cost < temp4.Cost) {
			//					removefunction(tentInt);
			//					call Tentative.pushfront(temp3); 
			//				}
		//				}
		//			}
		//			else
			//		{
			//			for(k = 0; k < temp3.NeighborsLength; k++)
			//			{
			//				if(temp3.Neighbors[k] == temp2.Node)
			//				{
				//				temp3.Next = temp2.Node;
				//				if(!call Tentative.isEmpty())
				//				{
				//					for(m = 0; m < call Tentative.size(); m++)
				//					{
				//						temp4 = call Tentative.get(k);
				//						if(temp4.Dest == temp3.Dest)
				//						{
				//							inTentList = TRUE;
				//							tentInt = m;
				//						}
				//					}
				//				}
				//				if(!call Confirmed.isEmpty())
				//				{
				//					for(m = 0; m < call Confirmed.size(); m++)
				//					{
				//						temp4 = call Confirmed.get(m);
				//						if(temp4.Dest == temp3.Dest)
				//						{
				//							inConList = TRUE;
				//						}
				//					}
				//				}
				//				if(!inTentList && !inConList)
				//				{
				//					call Tentative.pushfront(temp3);
				//				}
				//				else if(inTentList)
				//				{
				//					temp4 = call Tentative.get(tentInt);
				//					if(temp3.Cost < temp4.Cost)
				//					{
				//						removefunction(tentInt);
				//						call Tentative.pushfront(temp3);
				//					}
				//				}
				//			}
				//		}
				//	}
				}	
		//		else if (temp.Dest != TOS_NODE_ID) {
		//			for (k = 0; k < temp.NeighborsLength; k++) { 
		//				if (temp.Neighbors[k] == temp2.Node) {
		//					for(m = 0; m < temp.NeighborsLength; m++)
		//					{
		//						if(temp.Neighbors[m] == temp3.Dest && temp.Neighbors[m] != temp2.Node)
		//						{
		//							temp3.Next = temp2.Node;
		//							if (!call Tentative.isEmpty()) {
		//								for (l = 0; l < call Tentative.size(); l++) {
		//									temp4 = call Tentative.get(l);
		//									if (temp4.Dest == temp3.Dest) {
		//										inTentList = TRUE;
		//										tentInt = l;
		//									}
		//								}
		//							}
		//							if (!call Confirmed.isEmpty()) {
		//								for (l = 0; l < call Confirmed.size(); l++) {
			//								temp4 = call Confirmed.get(l);
			//								if (temp4.Dest == temp3.Dest) {
		//										inConList = TRUE;
		//									}
		//								}
		//							} 
		//							if (!inTentList && !inConList) {
		//								call Tentative.pushfront(temp3);
		//							}
		//							else if (inTentList) {
		//								temp4 = call Tentative.get(tentInt);
			//							if (temp3.Cost < temp4.Cost) {
			//								removefunction(tentInt);
			//								call Tentative.pushfront(temp3);
			//							}	
			//						}
			//					}
			//				}
			//			}									
			//		}	
			//	} 
		//	}
		//}
		//if (call Tentative.isEmpty()) {
		//	return;
		//}
		//else {
			//minCost = 65535;
			//for (k = 0; k < call Tentative.size(); k++) {
			//	minTemp = call Tentative.get(k);
			//	if (minTemp.Cost < minCost) {
			//		minCost = minTemp.Cost;
			//		minInt = k;
		//		} 
			//}
		//	minTemp = call Tentative.get(minInt);
		//	removefunction(minInt);
		//	algorithm(minTemp.Dest, minTemp.Cost, minTemp.Next, (uint8_t*) minTemp.Neighbors, minTemp.NeighborsLength);			
	//	}		
	//}
}
