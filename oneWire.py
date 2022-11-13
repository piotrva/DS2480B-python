import serial
import time
import crc8


class DS2480B:
    WRITE_FUNCTION = 0
    READ_FUNCTION = 1

    # Mode Commands
    MODE_DATA = 0xE1
    MODE_COMMAND = 0xE3
    MODE_STOP_PULSE = 0xF1

    # Return byte value
    RB_CHIPID_MASK = 0x1C
    RB_RESET_MASK = 0x03
    RB_1WIRESHORT = 0x00
    RB_PRESENCE = 0x01
    RB_ALARMPRESENCE = 0x02
    RB_NOPRESENCE = 0x03

    RB_BIT_MASK = 0x03
    RB_BIT_ONE = 0x03
    RB_BIT_ZERO = 0x00

    # Masks for all bit ranges
    CMD_MASK = 0x80
    FUNCTSEL_MASK = 0x60
    BITPOL_MASK = 0x10
    SPEEDSEL_MASK = 0x0C
    MODSEL_MASK = 0x02
    PARMSEL_MASK = 0x70
    PARMSET_MASK = 0x0E

    # Command or config bit
    CMD_COMM = 0x81
    CMD_CONFIG = 0x01

    # Function select bits
    FUNCTSEL_BIT = 0x00
    FUNCTSEL_SEARCHON = 0x30
    FUNCTSEL_SEARCHOFF = 0x20
    FUNCTSEL_RESET = 0x40
    FUNCTSEL_CHMOD = 0x60

    # Bit polarity/Pulse voltage bits
    BITPOL_ONE = 0x10
    BITPOL_ZERO = 0x00
    BITPOL_5V = 0x00
    BITPOL_12V = 0x10

    # One Wire speed bits
    SPEEDSEL_STD = 0x00
    SPEEDSEL_FLEX = 0x04
    SPEEDSEL_OD = 0x08
    SPEEDSEL_PULSE = 0x0C

    # Data/Command mode select bits
    MODSEL_DATA = 0x00
    MODSEL_COMMAND = 0x02

    # 5V Follow Pulse select bits
    PRIME5V_TRUE = 0x02
    PRIME5V_FALSE = 0x00

    # Parameter select bits
    PARMSEL_PARMREAD = 0x00
    PARMSEL_SLEW = 0x10
    PARMSEL_12VPULSE = 0x20
    PARMSEL_5VPULSE = 0x30
    PARMSEL_WRITE1LOW = 0x40
    PARMSEL_SAMPLEOFFSET = 0x50
    PARMSEL_ACTIVEPULLUPTIME = 0x60
    PARMSEL_BAUDRATE = 0x70

    # Pull down slew rate.
    PARMSET_Slew15Vus = 0x00
    PARMSET_Slew2p2Vus = 0x02
    PARMSET_Slew1p65Vus = 0x04
    PARMSET_Slew1p37Vus = 0x06
    PARMSET_Slew1p1Vus = 0x08
    PARMSET_Slew0p83Vus = 0x0A
    PARMSET_Slew0p7Vus = 0x0C
    PARMSET_Slew0p55Vus = 0x0E

    # 12V programming pulse time table
    PARMSET_32us = 0x00
    PARMSET_64us = 0x02
    PARMSET_128us = 0x04
    PARMSET_256us = 0x06
    PARMSET_512us = 0x08
    PARMSET_1024us = 0x0A
    PARMSET_2048us = 0x0C
    PARMSET_infinite = 0x0E

    # 5V strong pull up pulse time table
    PARMSET_16p4ms = 0x00
    PARMSET_65p5ms = 0x02
    PARMSET_131ms = 0x04
    PARMSET_262ms = 0x06
    PARMSET_524ms = 0x08
    PARMSET_1p05s = 0x0A
    PARMSET_dynamic = 0x0C
    # PARMSET_infinite = 0x0E

    # Write 1 low time
    PARMSET_Write8us = 0x00
    PARMSET_Write9us = 0x02
    PARMSET_Write10us = 0x04
    PARMSET_Write11us = 0x06
    PARMSET_Write12us = 0x08
    PARMSET_Write13us = 0x0A
    PARMSET_Write14us = 0x0C
    PARMSET_Write15us = 0x0E

    # Data sample offset and Write 0 recovery time
    PARMSET_SampOff3us = 0x00
    PARMSET_SampOff4us = 0x02
    PARMSET_SampOff5us = 0x04
    PARMSET_SampOff6us = 0x06
    PARMSET_SampOff7us = 0x08
    PARMSET_SampOff8us = 0x0A
    PARMSET_SampOff9us = 0x0C
    PARMSET_SampOff10us = 0x0E

    # Active pull up on time
    PARMSET_PullUp0p0us = 0x00
    PARMSET_PullUp0p5us = 0x02
    PARMSET_PullUp1p0us = 0x04
    PARMSET_PullUp1p5us = 0x06
    PARMSET_PullUp2p0us = 0x08
    PARMSET_PullUp2p5us = 0x0A
    PARMSET_PullUp3p0us = 0x0C
    PARMSET_PullUp3p5us = 0x0E

    # Baud rate bits
    PARMSET_9600 = 0x00
    PARMSET_19200 = 0x02
    PARMSET_57600 = 0x04
    PARMSET_115200 = 0x06

    # DS2480B program voltage available
    DS2480BPROG_MASK = 0x20

    # mode bit flags
    MODE_NORMAL = 0x00
    MODE_OVERDRIVE = 0x01
    MODE_STRONG5 = 0x02
    MODE_PROGRAM = 0x04
    MODE_BREAK = 0x08

    MAX_BAUD = PARMSET_115200

    def __init__(self, port, timeout=3):
        self.dev = serial.Serial(port=port, timeout=timeout)
        # search state
        self.ROM_NO = [0] * 8
        self.LastDiscrepancy = 0
        self.LastFamilyDiscrepancy = 0
        self.LastDeviceFlag = 0
        self.crc8 = crc8.crc8()

        # DS2480B state
        self.ULevel = 0  # 1-Wire level
        self.UBaud = 0  # baud rate
        self.UMode = 0  # command or data mode state
        self.USpeed = 0  # 1-Wire communication speed
        self.ALARM_RESET_COMPLIANCE = False  # flag for DS1994/DS2404 'special' reset

    # ---------------------------------------------------------------------------
    # -------- Basic 1-Wire functions
    # ---------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    # Reset all of the devices on the 1-Wire Net and return the result.
    #
    # Returns: TRUE(1):  presense pulse(s) detected, device(s) reset
    #          FALSE(0): no presense pulses detected
    #
    # WARNING: Without setting the above global (FAMILY_CODE_04_ALARM_TOUCHRESET_COMPLIANCE)
    #          to TRUE, this routine will not function correctly on some
    #          Alarm reset types of the DS1994/DS1427/DS2404 with
    #          Rev 1,2, and 3 of the DS2480/DS2480B.
    #
    #
    def OWReset(self):
        # readbuffer = list()
        sendpacket = list()

        # make sure normal level
        self.OWLevel(self.MODE_NORMAL)

        # check for correct mode
        if self.UMode != self.MODSEL_COMMAND:
            self.UMode = self.MODSEL_COMMAND
            sendpacket.append(self.MODE_COMMAND)

        # construct the command
        sendpacket.append(self.CMD_COMM | self.FUNCTSEL_RESET | self.USpeed)

        # flush the buffers
        self.dev.flush()

        # send the packet
        self.dev.write(sendpacket)
        # read back the 1 byte response
        readbuffer = self.dev.read(1)
        if len(readbuffer) == 1:
            # check for special reset
            if self.ALARM_RESET_COMPLIANCE:
                time.sleep(0.005)  # delay 5 ms to give DS1994 enough time
                self.dev.flush()
                return True

            # make sure this byte looks like a reset byte
            if (((readbuffer[0] & self.RB_RESET_MASK) == self.RB_PRESENCE) or
                    ((readbuffer[0] & self.RB_RESET_MASK) == self.RB_ALARMPRESENCE)):
                return True

        # an error occured so re-sync with DS2480B
        self.DS2480B_Detect()

        return False

    # --------------------------------------------------------------------------
    # Send 1 bit of communication to the 1-Wire Net.
    # The parameter 'sendbit' least significant bit is used.
    #
    # 'sendbit' - 1 bit to send (least significant byte)
    #
    def OWWriteBit(self, sendbit):
        self.OWTouchBit(sendbit)

    # --------------------------------------------------------------------------
    # Send 1 bit of read communication to the 1-Wire Net and and return the
    # result 1 bit read from the 1-Wire Net.
    #
    # Returns:  1 bit read from 1-Wire Net
    #
    def OWReadBit(self):
        return self.OWTouchBit(0x01)

    # --------------------------------------------------------------------------
    # Send 1 bit of communication to the 1-Wire Net and return the
    # result 1 bit read from the 1-Wire Net.  The parameter 'sendbit'
    # least significant bit is used and the least significant bit
    # of the result is the return bit.
    #
    # 'sendbit' - the least significant bit is the bit to send
    #
    # Returns: 0:   0 bit read from sendbit
    #          1:   1 bit read from sendbit
    #
    def OWTouchBit(self, sendbit):
        # readbuffer = list()
        sendpacket = list()

        # make sure normal level
        self.OWLevel(self.MODE_NORMAL)

        # check for correct mode
        if self.UMode != self.MODSEL_COMMAND:
            self.UMode = self.MODSEL_COMMAND
            sendpacket.append(self.MODE_COMMAND)

        # construct the command
        sendpacket.append(self.BITPOL_ONE if sendbit != 0 else self.BITPOL_ZERO)
        sendpacket[-1] |= self.CMD_COMM | self.FUNCTSEL_BIT | self.USpeed

        # flush the buffers
        self.dev.flush()

        # send the packet
        self.dev.write(sendpacket)
        # read back the response
        readbuffer = self.dev.read(1)
        if len(readbuffer) == 1:
            # interpret the response
            if ((readbuffer[0] & 0xE0) == 0x80) and ((readbuffer[0] & self.RB_BIT_MASK) == self.RB_BIT_ONE):
                return 1
            else:
                return 0

        # an error occured so re-sync with DS2480B
        self.DS2480B_Detect()

        return 0

    # --------------------------------------------------------------------------
    # Send 8 bits of communication to the 1-Wire Net and verify that the
    # 8 bits read from the 1-Wire Net is the same (write operation).
    # The parameter 'sendbyte' least significant 8 bits are used.
    #
    # 'sendbyte' - 8 bits to send (least significant byte)
    #
    # Returns:  TRUE: bytes written and echo was the same
    #           FALSE: echo was not the same
    #
    def OWWriteByte(self, sendbyte):
        recbyte = self.OWTouchByte(sendbyte)
        return recbyte == sendbyte

    # --------------------------------------------------------------------------
    # Send 8 bits of read communication to the 1-Wire Net and and return the
    # result 8 bits read from the 1-Wire Net.
    #
    # Returns:  8 bits read from 1-Wire Net
    #
    def OWReadByte(self):
        return self.OWTouchByte(0xFF)

    def OWReadBlock(self, num):
        readbuffer = list()
        for i in range(num):
            readbuffer.append(self.OWReadByte())
        return readbuffer

    # --------------------------------------------------------------------------
    # Send 8 bits of communication to the 1-Wire Net and return the
    # result 8 bits read from the 1-Wire Net.  The parameter 'sendbyte'
    # least significant 8 bits are used and the least significant 8 bits
    # of the result is the return byte.
    #
    # 'sendbyte' - 8 bits to send (least significant byte)
    #
    # Returns:  8 bits read from sendbyte
    #
    def OWTouchByte(self, sendbyte):
        # readbuffer = list()
        sendpacket = list()

        # make sure normal level
        self.OWLevel(self.MODE_NORMAL)

        # check for correct mode
        if self.UMode != self.MODSEL_DATA:
            self.UMode = self.MODSEL_DATA
            sendpacket.append(self.MODE_DATA)

        # add the byte to send
        sendpacket.append(sendbyte)

        # check for duplication of data that looks like COMMAND mode
        if sendbyte == self.MODE_COMMAND:
            sendpacket.append(sendbyte)

        # flush the buffers
        self.dev.flush()

        # send the packet
        self.dev.write(sendpacket)
        # read back the 1 byte response
        readbuffer = self.dev.read(1)
        if len(readbuffer) == 1:
            # return the response
            return readbuffer[0]

        # an error occured so re-sync with DS2480B
        self.DS2480B_Detect()

        return 0

    # --------------------------------------------------------------------------
    # The 'OWBlock' transfers a block of data to and from the
    # 1-Wire Net. The result is returned in the same buffer.
    #
    # 'tran_buf' - pointer to a block of unsigned
    #              chars of length 'tran_len' that will be sent
    #              to the 1-Wire Net
    #
    # Returns:   TRUE (1) : If the buffer transfer was succesful.
    #            FALSE (0): If an error occured.
    #
    #  The maximum tran_length is (160)
    #
    def OWBlock(self, tran_buf):
        sendpacket = list()

        # check for a block too big
        if len(tran_buf) > 160:
            return False

        # construct the packet to send to the DS2480B
        # check for correct mode
        if self.UMode != self.MODSEL_DATA:
            self.UMode = self.MODSEL_DATA
            sendpacket.append(self.MODE_DATA)

        # add the bytes to send
        for b in tran_buf:
            sendpacket.append(b)
            # duplicate data that looks like COMMAND mode
            if b == self.MODE_COMMAND:
                sendpacket.append(b)

        # flush the buffers
        self.dev.flush()

        # send the packet
        self.dev.write(sendpacket)
        # read back the response
        readbuffer = self.dev.read(len(tran_buf))
        if len(readbuffer) == len(tran_buf):
            return True

        # an error occured so re-sync with DS2480B
        self.DS2480B_Detect()

        return False

    # --------------------------------------------------------------------------
    # Find the 'first' devices on the 1-Wire bus
    # Return TRUE  : device found, ROM number in ROM_NO buffer
    #        FALSE : no device present
    #
    def OWFirst(self):
        # reset the search state
        self.LastDiscrepancy = 0
        self.LastDeviceFlag = False
        self.LastFamilyDiscrepancy = 0

        return self.OWSearch()

    # --------------------------------------------------------------------------
    # Find the 'next' devices on the 1-Wire bus
    # Return TRUE  : device found, ROM number in ROM_NO buffer
    #        FALSE : device not found, end of search
    #
    def OWNext(self):
        # leave the search state alone
        return self.OWSearch()

    # --------------------------------------------------------------------------
    # Verify the device with the ROM number in ROM_NO buffer is present.
    # Return TRUE  : device verified present
    #        FALSE : device not present
    #
    def OWVerify(self):
        rslt = False

        # keep a backup copy of the current state
        rom_backup = list(self.ROM_NO)
        ld_backup = int(self.LastDiscrepancy)
        ldf_backup = bool(self.LastDeviceFlag)
        lfd_backup = int(self.LastFamilyDiscrepancy)

        # set search to find the same device
        self.LastDiscrepancy = 64
        self.LastDeviceFlag = False

        if self.OWSearch():
            # check if same device found
            rslt = rom_backup == self.ROM_NO

        # restore the search state
        self.ROM_NO = list(rom_backup)
        self.LastDiscrepancy = int(ld_backup)
        self.LastDeviceFlag = bool(ldf_backup)
        self.LastFamilyDiscrepancy = int(lfd_backup)

        # return the result of the verify
        return rslt

    # --------------------------------------------------------------------------
    # Setup the search to find the device type 'family_code' on the next call
    # to OWNext() if it is present.
    #
    def OWTargetSetup(self, family_code):
        # set the search state to find SearchFamily type devices
        self.ROM_NO[0] = family_code
        self.ROM_NO[1:] = [0] * 7
        self.LastDiscrepancy = 64
        self.LastFamilyDiscrepancy = 0
        self.LastDeviceFlag = False

    # --------------------------------------------------------------------------
    # Setup the search to skip the current device type on the next call
    # to OWNext().
    #
    def OWFamilySkipSetup(self):
        # set the Last discrepancy to last family discrepancy
        self.LastDiscrepancy = self.LastFamilyDiscrepancy

        # clear the last family discrpepancy
        self.LastFamilyDiscrepancy = 0

        # check for end of list
        if self.LastDiscrepancy == 0:
            self.LastDeviceFlag = True

    # --------------------------------------------------------------------------
    # The 'OWSearch' function does a general search.  This function
    # continues from the previos search state. The search state
    # can be reset by using the 'OWFirst' function.
    # This function contains one parameter 'alarm_only'.
    # When 'alarm_only' is TRUE (1) the find alarm command
    # 0xEC is sent instead of the normal search command 0xF0.
    # Using the find alarm command 0xEC will limit the search to only
    # 1-Wire devices that are in an 'alarm' state.
    #
    # Returns:   TRUE (1) : when a 1-Wire device was found and it's
    #                       Serial Number placed in the global ROM
    #            FALSE (0): when no new device was found.  Either the
    #                       last search was the last device or there
    #                       are no devices on the 1-Wire Net.
    #
    def OWSearch(self, alarm_only=False):
        sendpacket = list()
        tmp_rom = [0] * 8

        # if the last call was the last one
        if self.LastDeviceFlag:
            # reset the search
            self.LastDiscrepancy = 0
            self.LastDeviceFlag = False
            self.LastFamilyDiscrepancy = 0
            return False

        # reset the 1-wire
        # if there are no parts on 1-wire, return FALSE
        if not self.OWReset():
            # reset the search
            self.LastDiscrepancy = 0
            self.LastFamilyDiscrepancy = 0
            return False

        # build the command stream
        # call a function that may add the change mode command to the buff
        # check for correct mode
        if self.UMode != self.MODSEL_DATA:
            self.UMode = self.MODSEL_DATA
            sendpacket.append(self.MODE_DATA)

        # search command
        if alarm_only:
            sendpacket.append(0xEC)
        else:
            sendpacket.append(0xF0)

        # change back to command mode
        self.UMode = self.MODSEL_COMMAND
        sendpacket.append(self.MODE_COMMAND)

        # search mode on
        sendpacket.append(self.CMD_COMM | self.FUNCTSEL_SEARCHON | self.USpeed)

        # change back to data mode
        self.UMode = self.MODSEL_DATA
        sendpacket.append(self.MODE_DATA)

        # set the temp Last Discrepancy to 0
        last_zero = 0

        # add the 16 bytes of the search
        sendpacketROM = [0] * 16

        # only modify bits if not the first search
        if self.LastDiscrepancy != 0:
            # set the bits in the added buffer
            for i in range(64):
                # before last discrepancy
                if i < (self.LastDiscrepancy - 1):
                    rbit = self.bitacc(self.READ_FUNCTION, 0, i, self.ROM_NO)
                    self.bitacc(self.WRITE_FUNCTION, rbit, (i * 2 + 1), sendpacketROM)
                # at last discrepancy
                elif i == (self.LastDiscrepancy - 1):
                    self.bitacc(self.WRITE_FUNCTION, 1, (i * 2 + 1), sendpacketROM)
                # after last discrepancy so leave zeros

        # add the 16 bytes of the search
        sendpacket.extend(sendpacketROM)

        # change back to command mode
        self.UMode = self.MODSEL_COMMAND
        sendpacket.append(self.MODE_COMMAND)

        # search OFF command
        sendpacket.append(self.CMD_COMM | self.FUNCTSEL_SEARCHOFF | self.USpeed)

        # flush the buffers
        self.dev.flush()

        # send the packet
        self.dev.write(sendpacket)

        # read back the 1 byte response
        readbuffer = list(self.dev.read(17))
        if len(readbuffer) == 17:
            readbuffer.pop(0)
            # interpret the bit stream
            for i in range(64):
                # get the ROM bit
                rbit = self.bitacc(self.READ_FUNCTION, 0, (i * 2 + 1), readbuffer)
                self.bitacc(self.WRITE_FUNCTION, rbit, i, tmp_rom)
                # check LastDiscrepancy
                if self.bitacc(self.READ_FUNCTION, 0, (i * 2), readbuffer) == 1 and\
                        self.bitacc(self.READ_FUNCTION, 0, (i * 2 + 1), readbuffer) == 0:
                    last_zero = i + 1
                    # check LastFamilyDiscrepancy
                    if i < 8:
                        self.LastFamilyDiscrepancy = i + 1

            # do dowcrc
            self.crc8.reset()
            for i in range(8):
                self.crc8.docrc8(tmp_rom[i])

            # check results
            if (self.crc8.read() != 0) or (self.LastDiscrepancy == 63) or (tmp_rom[0] == 0):
                # error during search
                # reset the search
                self.LastDiscrepancy = 0
                self.LastDeviceFlag = False
                self.LastFamilyDiscrepancy = 0
                return False
            # successful search
            else:
                # set the last discrepancy
                self.LastDiscrepancy = last_zero

                # check for last device
                if self.LastDiscrepancy == 0:
                    self.LastDeviceFlag = True

                # copy the ROM to the buffer
                self.ROM_NO = list(tmp_rom)

                return True

        # an error occured so re-sync with DS2480B
        self.DS2480B_Detect()

        # reset the search
        self.LastDiscrepancy = 0
        self.LastDeviceFlag = False
        self.LastFamilyDiscrepancy = 0

        return False

    # ---------------------------------------------------------------------------
    # -------- Extended 1-Wire functions
    # ---------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    # Set the 1-Wire Net communucation speed.
    #
    # 'new_speed' - new speed defined as
    #                MODE_NORMAL     0x00
    #                MODE_OVERDRIVE  0x01
    #
    # Returns:  current 1-Wire Net speed
    #
    def OWSpeed(self, new_speed):
        sendpacket = list()
        rt = False

        # check if change from current mode
        if ((new_speed == self.MODE_OVERDRIVE) and (self.USpeed != self.SPEEDSEL_OD)) or (
                (new_speed == self.MODE_NORMAL) and (self.USpeed != self.SPEEDSEL_FLEX)):
            if new_speed == self.MODE_OVERDRIVE:
                # if overdrive then switch to 115200 baud
                if self.DS2480B_ChangeBaud(self.MAX_BAUD) == self.MAX_BAUD:
                    self.USpeed = self.SPEEDSEL_OD
                    rt = True
            elif new_speed == self.MODE_NORMAL:
                # else normal so set to 9600 baud
                if self.DS2480B_ChangeBaud(self.PARMSET_9600) == self.PARMSET_9600:
                    self.USpeed = self.SPEEDSEL_FLEX
                    rt = True

            # if baud rate is set correctly then change DS2480B speed
            if rt:
                # check for correct mode
                if self.UMode != self.MODSEL_COMMAND:
                    self.UMode = self.MODSEL_COMMAND
                    sendpacket.append(self.MODE_COMMAND)

                # proceed to set the DS2480B communication speed
                sendpacket.append(self.CMD_COMM | self.FUNCTSEL_SEARCHOFF | self.USpeed)

                # send the packet
                self.dev.write(sendpacket)

        # return the current speed
        if self.USpeed == self.SPEEDSEL_OD:
            return self.MODE_OVERDRIVE
        else:
            return self.MODE_NORMAL

    # --------------------------------------------------------------------------
    # Set the 1-Wire Net line level.  The values for new_level are
    # as follows:
    #
    # 'new_level' - new level defined as
    #                MODE_NORMAL     0x00
    #                MODE_STRONG5    0x02
    #
    # Returns:  current 1-Wire Net level
    #
    def OWLevel(self, new_level):
        sendpacket = list()
        rt = False
        # seems to have no effect in AN192...
        # docheck = False

        # check if need to change level
        if new_level != self.ULevel:
            # check for correct mode
            if self.UMode != self.MODSEL_COMMAND:
                self.UMode = self.MODSEL_COMMAND
                sendpacket.append(self.MODE_COMMAND)

            # check if just putting back to normal
            if new_level == self.MODE_NORMAL:
                # check for disable strong pullup step
                # seems to have no effect in AN192...
                # if self.ULevel == self.MODE_STRONG5:
                #     docheck = True

                # stop pulse command
                sendpacket.append(self.MODE_STOP_PULSE)

                # add the command to begin the pulse WITHOUT prime
                sendpacket.append(
                    self.CMD_COMM | self.FUNCTSEL_CHMOD | self.SPEEDSEL_PULSE | self.BITPOL_5V | self.PRIME5V_FALSE)

                # stop pulse command
                sendpacket.append(self.MODE_STOP_PULSE)

                # flush the buffers
                self.dev.flush()

                # send the packet
                self.dev.write(sendpacket)
                # read back the 1 byte response
                readbuffer = self.dev.read(2)
                if len(readbuffer) == 2:
                    # check response byte
                    if ((readbuffer[0] & 0xE0) == 0xE0) and ((readbuffer[1] & 0xE0) == 0xE0):
                        rt = True
                        self.ULevel = self.MODE_NORMAL
            # set new level
            else:
                # set the SPUD time value
                sendpacket.append(self.CMD_CONFIG | self.PARMSEL_5VPULSE | self.PARMSET_infinite)
                # add the command to begin the pulse
                sendpacket.append(self.CMD_COMM | self.FUNCTSEL_CHMOD | self.SPEEDSEL_PULSE | self.BITPOL_5V)

                # flush the buffers
                self.dev.flush()

                # send the packet
                self.dev.write(sendpacket)
                # read back the 1 byte response from setting time limit
                readbuffer = self.dev.read(1)
                if len(readbuffer) == 1:
                    # check response byte
                    if (readbuffer[0] & 0x81) == 0:
                        self.ULevel = new_level
                        rt = True

            # if lost communication with DS2480B then reset
            if not rt:
                self.DS2480B_Detect()

        # return the current level
        return self.ULevel

    # --------------------------------------------------------------------------
    # This procedure creates a fixed 480 microseconds 12 volt pulse
    # on the 1-Wire Net for programming EPROM iButtons.
    #
    # Returns:  TRUE  successful
    #           FALSE program voltage not available
    #
    def OWProgramPulse(self):
        sendpacket = list()

        # make sure normal level
        self.OWLevel(self.MODE_NORMAL)

        # check for correct mode
        if self.UMode != self.MODSEL_COMMAND:
            self.UMode = self.MODSEL_COMMAND
            sendpacket.append(self.MODE_COMMAND)

        # set the SPUD time value
        sendpacket.append(self.CMD_CONFIG | self.PARMSEL_12VPULSE | self.PARMSET_512us)

        # pulse command
        sendpacket.append(self.CMD_COMM | self.FUNCTSEL_CHMOD | self.BITPOL_12V | self.SPEEDSEL_PULSE)

        # flush the buffers
        self.dev.flush()

        # send the packet
        self.dev.write(sendpacket)
        # read back the 2 byte response
        readbuffer = self.dev.read(2)
        if len(readbuffer) == 2:
            # check response byte
            if ((readbuffer[0] | self.CMD_CONFIG) == (self.CMD_CONFIG | self.PARMSEL_12VPULSE | self.PARMSET_512us)) and \
                    ((readbuffer[1] & 0xFC) == (0xFC & (self.CMD_COMM | self.FUNCTSEL_CHMOD | self.BITPOL_12V | self.SPEEDSEL_PULSE))):
                return True

        # an error occured so re-sync with DS2480B
        self.DS2480B_Detect()

        return False

    # --------------------------------------------------------------------------
    # Send 8 bits of communication to the 1-Wire Net and verify that the
    # 8 bits read from the 1-Wire Net is the same (write operation).
    # The parameter 'sendbyte' least significant 8 bits are used.  After the
    # 8 bits are sent change the level of the 1-Wire net.
    #
    # 'sendbyte' - 8 bits to send (least significant bit)
    #
    # Returns:  TRUE: bytes written and echo was the same, strong pullup now on
    #           FALSE: echo was not the same
    #
    def OWWriteBytePower(self, sendbyte):
        sendpacket = list()
        rt = False

        # check for correct mode
        if self.UMode != self.MODSEL_COMMAND:
            self.UMode = self.MODSEL_COMMAND
            sendpacket.append(self.MODE_COMMAND)

        # set the SPUD time value
        sendpacket.append(self.CMD_CONFIG | self.PARMSEL_5VPULSE | self.PARMSET_infinite)

        # construct the stream to include 8 bit commands with the last one
        # enabling the strong-pullup
        temp_byte = int(sendbyte)
        for i in range(8):
            sendpacket.append((self.BITPOL_ONE if (
                        temp_byte & 0x01) else self.BITPOL_ZERO) | self.CMD_COMM | self.FUNCTSEL_BIT | self.USpeed | (
                                  self.PRIME5V_TRUE if (i == 7) else self.PRIME5V_FALSE))
            temp_byte >>= 1

        # flush the buffers
        self.dev.flush()

        # send the packet
        self.dev.write(sendpacket)
        # read back the 9 byte response from setting time limit
        readbuffer = self.dev.read(9)
        if len(readbuffer) == 9:
            # check response
            if (readbuffer[0] & 0x81) == 0:
                # indicate the port is now at power delivery
                self.ULevel = self.MODE_STRONG5

                # reconstruct the echo byte
                temp_byte = 0
                for i in range(8):
                    temp_byte >>= 1
                    temp_byte |= 0x80 if (readbuffer[i + 1] & 0x01) else 0

                if temp_byte == sendbyte:
                    rt = True

        # if lost communication with DS2480B then reset
        if not rt:
            self.DS2480B_Detect()

        return rt

    # --------------------------------------------------------------------------
    # Send 1 bit of communication to the 1-Wire Net and verify that the
    # response matches the 'applyPowerResponse' bit and apply power delivery
    # to the 1-Wire net.  Note that some implementations may apply the power
    # first and then turn it off if the response is incorrect.
    #
    # 'applyPowerResponse' - 1 bit response to check, if correct then start
    #                        power delivery
    #
    # Returns:  TRUE: bit written and response correct, strong pullup now on
    #           FALSE: response incorrect
    #
    def OWReadBitPower(self, applyPowerResponse):
        sendpacket = list()
        rt = False

        # check for correct mode
        if self.UMode != self.MODSEL_COMMAND:
            self.UMode = self.MODSEL_COMMAND
            sendpacket.append(self.MODE_COMMAND)

        # set the SPUD time value
        sendpacket.append(self.CMD_CONFIG | self.PARMSEL_5VPULSE | self.PARMSET_infinite)

        # enabling the strong-pullup after bit
        sendpacket.append(self.BITPOL_ONE | self.CMD_COMM | self.FUNCTSEL_BIT | self.USpeed | self.PRIME5V_TRUE)
        # flush the buffers
        self.dev.flush()

        # send the packet
        self.dev.write(sendpacket)
        # read back the 2 byte response from setting time limit
        readbuffer = self.dev.read(2)
        if len(readbuffer) == 2:
            # check response to duration set
            if (readbuffer[0] & 0x81) == 0:
                # indicate the port is now at power delivery
                self.ULevel = self.MODE_STRONG5

                # check the response bit
                if (readbuffer[1] & 0x01) == applyPowerResponse:
                    rt = True
                else:
                    self.OWLevel(self.MODE_NORMAL)
                return rt

        # if lost communication with DS2480B then reset
        if not rt:
            self.DS2480B_Detect()

        return rt

    # ---------------------------------------------------------------------------
    # -------- DS2480B functions
    # ---------------------------------------------------------------------------

    # ---------------------------------------------------------------------------
    # Attempt to resyc and detect a DS2480B and set the FLEX parameters
    #
    # Returns:  TRUE  - DS2480B detected successfully
    #           FALSE - Could not detect DS2480B
    #
    def DS2480B_Detect(self):
        sendpacket = list()

        # reset modes
        self.UMode = self.MODSEL_COMMAND
        self.UBaud = self.PARMSET_9600
        self.USpeed = self.SPEEDSEL_FLEX

        # set the baud rate to 9600
        if self.UBaud == self.PARMSET_9600:
            self.dev.baudrate = 9600
        elif self.UBaud == self.PARMSET_19200:
            self.dev.baudrate = 19200
        elif self.UBaud == self.PARMSET_57600:
            self.dev.baudrate = 57600
        elif self.UBaud == self.PARMSET_115200:
            self.dev.baudrate = 115200
        else:
            self.dev.baudrate = 9600

        # send a break to reset the DS2480B
        self.dev.send_break()

        # delay to let line settle
        time.sleep(0.002)

        # flush the buffers
        self.dev.flush()

        # send the timing byte
        sendpacket.append(0xC1)
        self.dev.write(sendpacket)

        # delay to let line settle
        time.sleep(0.002)

        # set the FLEX configuration parameters
        sendpacket = list()
        # default PDSRC = 1.37Vus
        sendpacket.append(self.CMD_CONFIG | self.PARMSEL_SLEW | self.PARMSET_Slew1p37Vus)
        # default W1LT = 10us
        sendpacket.append(self.CMD_CONFIG | self.PARMSEL_WRITE1LOW | self.PARMSET_Write10us)
        # default DSO/WORT = 8us
        sendpacket.append(self.CMD_CONFIG | self.PARMSEL_SAMPLEOFFSET | self.PARMSET_SampOff8us)

        # construct the command to read the baud rate (to test command block)
        sendpacket.append(self.CMD_CONFIG | self.PARMSEL_PARMREAD | (self.PARMSEL_BAUDRATE >> 3))

        # also do 1 bit operation (to test 1-Wire block)
        sendpacket.append(self.CMD_COMM | self.FUNCTSEL_BIT | self.UBaud | self.BITPOL_ONE)

        # flush the buffers
        self.dev.flush()

        # send the packet
        self.dev.write(sendpacket)
        # read back the response
        readbuffer = self.dev.read(5)
        if len(readbuffer) == 5:
            # look at the baud rate and bit operation
            # to see if the response makes sense
            if ((readbuffer[3] & 0xF1) == 0x00) and \
                    ((readbuffer[3] & 0x0E) == self.UBaud) and \
                    ((readbuffer[4] & 0xF0) == 0x90) and \
                    ((readbuffer[4] & 0x0C) == self.UBaud):
                return True

        return False

    # ---------------------------------------------------------------------------
    # Change the DS2480B from the current baud rate to the new baud rate.
    #
    # 'newbaud' - the new baud rate to change to, defined as:
    #               PARMSET_9600     0x00
    #               PARMSET_19200    0x02
    #               PARMSET_57600    0x04
    #               PARMSET_115200   0x06
    #
    # Returns:  current DS2480B baud rate.
    #
    def DS2480B_ChangeBaud(self, newbaud):
        rt = False
        sendpacket = list()
        sendpacket2 = list()

        # see if diffenent then current baud rate
        if self.UBaud == newbaud:
            return self.UBaud
        else:
            # build the command packet
            # check for correct mode
            if self.UMode != self.MODSEL_COMMAND:
                self.UMode = self.MODSEL_COMMAND
                sendpacket.append(self.MODE_COMMAND)
            # build the command
            sendpacket.append(self.CMD_CONFIG | self.PARMSEL_BAUDRATE | newbaud)

            # flush the buffers
            self.dev.flush()

            # send the packet
            self.dev.write(sendpacket)
            # make sure buffer is flushed
            time.sleep(0.005)

            # change our baud rate
            self.UBaud = newbaud
            if self.UBaud == self.PARMSET_9600:
                self.dev.baudrate = 9600
            elif self.UBaud == self.PARMSET_19200:
                self.dev.baudrate = 19200
            elif self.UBaud == self.PARMSET_57600:
                self.dev.baudrate = 57600
            elif self.UBaud == self.PARMSET_115200:
                self.dev.baudrate = 115200
            else:
                self.dev.baudrate = 9600

            # wait for things to settle
            time.sleep(0.005)

            # build a command packet to read back baud rate
            sendpacket2.append(self.CMD_CONFIG | self.PARMSEL_PARMREAD | (self.PARMSEL_BAUDRATE >> 3))

            # flush the buffers
            self.dev.flush()

            # send the packet
            self.dev.write(sendpacket2)
            # read back the 1 byte response
            readbuffer = self.dev.read(1)
            if len(readbuffer) == 1:
                # verify correct baud
                if (readbuffer[0] & 0x0E) == (sendpacket[-1] & 0x0E):
                    rt = True

        # if lost communication with DS2480B then reset
        if not rt:
            self.DS2480B_Detect()

        return self.UBaud

    # ---------------------------------------------------------------------------
    # -------- Utility functions
    # ---------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    # Bit utility to read and write a bit in the buffer 'buf'.
    #
    # 'op'    - operation (1) to set and (0) to read
    # 'state' - set (1) or clear (0) if operation is write (1)
    # 'loc'   - bit number location to read or write
    # 'buf'   - pointer to array of bytes that contains the bit
    #           to read or write
    #
    # Returns: 1   if operation is set (1)
    #          0/1 state of bit number 'loc' if operation is reading
    #
    def bitacc(self, op, state, loc, buf):
        nbyt = int(loc / 8)
        nbit = loc - (nbyt * 8)

        if op == self.WRITE_FUNCTION:
            if state:
                buf[nbyt] |= (0x01 << nbit)
            else:
                buf[nbyt] &= ~(0x01 << nbit)

            return 1
        else:
            return (buf[nbyt] >> nbit) & 0x01

    def printROM(self):
        for i in range(7, -1, -1):
            print("{:02X}".format(self.ROM_NO[i]), end=' ')
        print()