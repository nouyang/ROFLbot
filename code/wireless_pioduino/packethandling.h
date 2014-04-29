/* packethandling.h -- handles everything about getting and sending packets to soar
   contains calcChecksum(), sendPkt(), receivePkt() */

//copied from pioneer.py edited for arduino
//see pg. 27 code/pioneer-manualv3.pdf
int calcCheckSum(byte data[]) {
  int c = 0;
  int i = 3;
  int n = data[2]-2;
  while (n>1) {
    c += (data[i]<<8) | (data[i+1]);
    c = c & 0xFFFF;
    n -= 2;
    i += 2;
  }
  if (n>0) {
    c = c ^ data[i];
  }
  return c;
}

/* Sends a packet to soar takes a list of data and the list's length
   this function adds a header and the checksum see pg 27 of
   code/pioneer-manualv3.pdf for more info on packet construction*/
void sendPkt(byte data[], size_t dataLength) {
  //3 for the header and 2 for checksum
  byte pktLength = 5+dataLength;
  byte pkt[pktLength];
  pkt[0] = 0xFA;
  pkt[1] = 0xFB;
  pkt[2] = dataLength+2;
  for (int i = 0; i < dataLength; i++) {
    pkt[3+i] = data[i];
  }
  int chk = calcCheckSum(pkt);
  pkt[pktLength-2] = chk>>8;
  pkt[pktLength-1] = chk&0xFF;
  for(int i = 0; i < pktLength; i++) {
    Serial1.write(pkt[i]);
    //block until the byte is sent increases packet reception
    Serial1.flush(); 
  }
  return;
}

/* Receives packets from soar. Takes a pointer to an array and fills it
   with the received data. See pg. 27 of code/pioneer-manualv3.pdf for
   packet construction details. */
void receivePkt(byte *data) {
  byte header[] = {0,0,0};
  boolean headerCheck = false;
  //get the header
  for(int i=0; i < 3; i++) {
    byte next = Serial1.peek();
    /* If the first part of a packet is dropped and the Arduino starts
       reading from the middle of subsequent packets it likely wont get
       back on track and all future reading will be incorrect. Peeking
       and checking the first byte, then throwing it away if it isn't
       right mitigates this issue and the robot connects wonderfully. */
    if(!headerCheck && next == 0xFA) {
      header[i] = Serial1.read();
      headerCheck = true;
    } else if(next == 255) {
      /* There should never ever be a 255 in the header.  Seeing one either
         means the rest of the packet was dropped, or the serial buffer
         was read faster than data was pushed into it.  Either way if we
         keep reading all the following packets will be parsed incorrectly */         
      return;
    } else if(headerCheck) {
      header[i] = Serial1.read();
    } else {
      Serial1.read();
      i--;
    }
  }

  //get the rest of the packet
  if (header[0] == 0xFA and header[1] == 0xFB) {
    //size of the recieved packet
    int receivedDataLength = header[2];
    unsigned long time = millis();
    int timeout = 100;
    //block until the whole packet gets here to prevent reading errors
    while(Serial1.available() < receivedDataLength) { 
      delayMicroseconds(100);
      if((millis()-time) > timeout) {
        data[0] = FAIL;
        return;
      }
    }
    for(int d=0; d < receivedDataLength; d++) {
      data[d] = Serial1.read();
    }
    
    //verify packet checksum
    byte pkt[receivedDataLength+sizeof(header)];
    for(int i=0; i < 3; i++) {
      pkt[i] = header[i];
    }
    for(int i=3; i < receivedDataLength+3; i++) {
      pkt[i] = data[i-3];
    }
    int crc = calcCheckSum(pkt);
    if(pkt[sizeof(pkt)-1]!=(crc&0xFF) || pkt[sizeof(pkt)-2]!=((crc>>8)&0xFF)) {
      memset(data,0,sizeof(data));
      data[0] = FAIL;
    }
  }
  return;
}
