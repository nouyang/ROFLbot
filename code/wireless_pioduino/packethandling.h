/* packethandling.h -- handles everything about getting and sending packets to soar
   contains calcChecksum(), sendPacket(), recvPacket() */

//copied from pioneer.py edited for arduino
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
   this function adds a header and the checksum */
void sendPacket(byte data[], size_t datalength) {
  byte pktlength = 5+datalength; //3 for the header and 2 for checksum
  byte pkt[pktlength];
  pkt[0] = 0xFA; //0 and 1 are constant headers
  pkt[1] = 0xFB;
  pkt[2] = datalength+2; //2 is length of data plus 2 for checksum
  for (int i = 0; i < datalength; i++) {
    pkt[3+i] = data[i];
  }
  int chk = calcCheckSum(pkt);
  pkt[pktlength-2] = chk>>8; //put the checksum at the end
  pkt[pktlength-1] = chk&0xFF;
  for(int i = 0; i < pktlength; i++) {
    Serial1.write(pkt[i]); //send it to soar!
    Serial1.flush(); //block until it's sent
  }
  return;
}

/* Receives packets from soar. Takes a pointer to an array and fills it
   with the received data. */
void recvPacket(byte *data) {
  byte header[] = {0,0,0};
  boolean headercheck = false;
  for(int i=0; i < 3; i++) { //get the header
    byte next = Serial1.peek();
    /*All of these checks seem redundant but it connects way better with them.
      Mostly, there's a lot of ways for a packet to fail and if it gets
      off track reading everything goes wrong. */
    if(!headercheck && next == 0xFA) {
      header[i] = Serial1.read();
      headercheck = true;
    } else if(headercheck) {
      header[i] = Serial1.read();
    } else if(next == 255) {
      /* if you see a 255 the serial buffer is empty or read to fast
         and we made it to this loop because of a bad or excessively
         slow packet this stops the loop */
      return;
    } else {
      i--;
    }
  }
  if (header[0] == 0xFA and header[1] == 0xFB) { //double check that it's actually a packet
    int recdatalength = header[2]; //size of the recieved packet
    unsigned long time = millis();
    int timeout = 100;
    while(Serial1.available() < recdatalength) { //block until the whole packet gets here
      delayMicroseconds(100);
      if((millis()-time) > timeout) { //taking too long? give up.
        data[0] = FAIL;
        return;
      }
    }
    for(int d=0; d < recdatalength; d++) { //populate the array
      data[d] = Serial1.read();
    }
    byte pkt[recdatalength+sizeof(header)]; //construct a silly packet to check the CRC
    for(int i=0; i < 3; i++) {
      pkt[i] = header[i];
    }
    for(int i=3; i < recdatalength+3; i++) {
      pkt[i] = data[i-3];
    }
    int crc = calcCheckSum(pkt);
    if(pkt[sizeof(pkt)-1]!=(crc&0xFF) || pkt[sizeof(pkt)-2]!=((crc>>8)&0xFF)) { //check the CRC
      memset(data,0,sizeof(data));
      data[0] = FAIL;
    }
  }
  return;
}
