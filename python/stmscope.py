"""Communication with NUCLEO boards.
Reads string from standard inputs and send them to the board using USB-serial 
inteface (dev/ttyACM*).
String can contain several commands, separated by semicolon.
The last command can be repeated by sending an upArrow sequence. 
"""
"""
Expected the following message structure from STM32:
  struct SERBIN{
  uint16_t l; //message length in bytes
  char d;     //data description, bits[0:1] number of bytes per item minus 1, bits[2:5]: number of channels minus 1
  char id;    //data ID, 'A' for ADC
  char payload[4+4*NCONV]; //2048 is good for REPEATS=128
  }
"""
__version__ = 'v0.2.5 2024-04-16'# fix case when pargs.graph is no None 
print(f'Version {__version__}')
#TODO: nADC should affect which ADC are being read by the server.

import sys, time, os
import serial
import threading
import numpy as np
import json

UpArrow = b'\x1b[A' #Up arrow sequence
LastCmd = ''
RX_overrun = False

def printTime(): return time.strftime("%m%d:%H%M%S")
def croppedText(txt, limit=200):
    if len(txt) > limit:
        txt = txt[:limit]+'...'
    return txt

def printi(msg): print(f'INF@{printTime()}: {msg}')

def printw(msg):
    msg = croppedText(msg)
    print(f'WRN@{printTime()}: {msg}')

def printe(msg):
    msg = croppedText(msg)
    print(f'ERR@{printTime()}: {msg}')

def _printv(msg, level=0):
    if pargs.verbose is None:
        return
    if len(pargs.verbose) >= level:
        print(msg)
def printv(msg):   _printv(msg, 0)
def printvv(msg):  _printv(msg, 1)

def b2i(buf):
    return int.from_bytes(buf, 'little')

def open_serdev():
    timeout = 1
    try:
        r = serial.Serial(pargs.port, pargs.baudrate, timeout=timeout)#, inter_byte_timeout=0.1)#write_timeout=.1) #parity=serial.PARITY_EVEN, rtscts=1)
    except serial.SerialException as e:
        ttyACM = '/dev/ttyACM'
        if pargs.port.startswith(ttyACM):
            suffix = pargs.port[-1]
            if suffix in '01':
                pargs.port = ttyACM + {'0':'1', '1':'0'}[suffix]
                try:
                    r = serial.Serial(pargs.port, pargs.baudrate, timeout=timeout)
                except:
                    printe('Could not open ttyACM0/1')
                    sys.exit(1)
                printw(f'Was not able to open {pargs.port}, using {pargs.port} instead')
        else:
            printe(f'Could not open {pargs.port}: {e}')
            sys.exit(1)
    return r

lastheader = None
#legalID = ['A']
adc_srate = 0.9999
feod = b'\x04\x00\x00E' #EndOfData, l=4, id='E'

def resync():
    ts = time.time()
    while 1:
        if time.time() - ts > 1:
            printw(f'Resync failed.')
            return False
        try:
            f = serdev.read(1)
            #print(f'rs:{b}, {feod[0]}')
            if f[0] != feod[0]:    continue
            e = serdev.read(1)
            if e[0] != feod[1]:    continue
            o = serdev.read(1)
            if o[0] != feod[2]:    continue
            #print(f'feo match: {f,e,o}')
            d = serdev.read(1)
            if d[0] != feod[3]:    continue
            break
        except Exception as e:
            printw(f'Exception in resync: {e}')
    printw(f'Resync OK.')
    return True

def get_data(cmd=None):
    """Execute command, if submitted and receive data.
    - Returns hdrID,reply from the frontend.
    - Returns None if no data.
    - Returns int error code if abnormal."""
    global lastheader, adc_srate, RX_overrun
    #if lastheader == None:
    if cmd:
        write_serdev(cmd)
    while 1:
        header = serdev.read(4)
        if len(header) != 4:
            #printv(f'No data: {header}')
            #serdev.flush()
            if cmd:
                time.sleep(0.01)
                printe('FEC not responding')
                return 1
            return None
        if header ==  feod:
            printvv(f'EndOfData message')
            continue
        if lastheader is not None and header != lastheader:
            printw(f'data shape changed: {header}')
            lastheader = header
        l = b2i(header[:2])
        if header[3] == 0:
            printe(f'Header wrong: {header}')
            if resync():
                continue
            return 2
        hdrNB = (header[2]&0x3) + 1
        hdrNCh = ((header[2]>>2)&0xf) + 1
        printvv(f'hdr:{header}, l:{l}, NB:{hdrNB}, NChannels: {hdrNCh}')
        samplesPerChannel = l//hdrNB//hdrNCh
        errmsg = None
        try:
            hdrID = header[3:4]
        except:
            printe(f'Number of bits is wrong: {header}')
            if resync():
                continue
            return 3
        #if hdrID not in legalID:
        #    printe(f'ID {header[3]} not supported')
        #    return
        if hdrID == b'A':# binary ADC data
            #printv(f'h: {header}')
            payload = serdev.read(l-4)
            #printvv(croppedText(f'payload: {payload}'))
            dtype = {1:np.uint8, 2:np.uint16, 4:np.uint32}.get(hdrNB)
            if dtype is None:
                printe(f'3-byte word is not supported: {header}')
                #serdev.flush()
                if resync():
                    continue
                return 4
            #print(f'dt: {dtype}')
            try:
                reply = np.frombuffer(payload,dtype\
                  = dtype).reshape(samplesPerChannel,hdrNCh).T
            except:
                printe(f'data[{len(payload)}] shape is wrong, channels: {hdrNCh}, width: {samplesPerChannel}')
                #serdev.flush()
                if resync():
                    continue
                return 5
            #print(f'reply {type(reply)}: {reply.shape}')
        elif hdrID == b'<':# JSON formatted reply
            payload = serdev.read(l-4)
            if not payload.startswith(b'{'):
                msg = payload.decode()
                if 'RX_overrun' in msg:
                    RX_overrun = True
                else:
                    RX_overrun = False
                #printi(f'msg: {msg}')
                if msg == 'OK':
                    print('',end='\n>')
                else:
                    printi(f'msg: {msg}')
                return hdrID,payload
            try:
                reply = json.loads(payload)
            except Exception as e:
                printe(f'Error {e} in JSON decoding of:\n{payload}')
                continue
            print(f'{reply}',end='\n>')
            # check if it was 'get adc_srate' command
            try:
                srate = reply['adc_srate']
                if isinstance(srate,int):
                    adc_srate = srate
                    print(f'ADC srate: {adc_srate} Hz')
                    plot.setLabel('bottom', 'Time', units='S')
            except:
                pass
        else:
            printe(f'ID {header[3]} not supported, header: {header}')
            if resync():
                continue
            return
        return hdrID,reply

curves = []
lasttime = time.time()
fps = 0
def read_serial():
    global lasttime,fps
    try:
        r = get_data()
        ct = time.time()
        if ct - lasttime >= 10:
            lasttime = ct
            txt = f'Frame rate: {round(fps/10,3)} Hz'
            if not pargs.quiet: printi(txt)
            if not pargs.graph == '':
                TextItem.setText(txt)
            fps = 0
        if r is None:
            if not pargs.quiet: 
                msg = 'no data'
                #if RX_overrun:
                #    msg += f', repeating `{LastCmd}`'
                #    write_serdev(LastCmd)
                printv(msg)
            return
        hdrID,data = r
        if hdrID != b'A':
            return

        # waveforms received
        fps += 1
        lx = len(data[0])
        x = np.arange(lx)/adc_srate
        printvv(f'Header: {hdrID}, data:[{len(data)}]:\n{data}')
        if pargs.graph == '':
            return
        if len(curves) == 0:
            for idx,row in enumerate(data):
                if pargs.graph is None or str(idx+1) in pargs.graph:
                    curve = pg.PlotCurveItem(pen=(idx,len(data)*1.3))
                    plot.addItem(curve)
                    curves.append((idx,curve))
                    legend.addItem(curve, f'ADC{idx+1}')
            plot.resize(900,600)
        for curveIdx,didxCurve in enumerate(curves):
            dataIdx,curve = didxCurve
            row = data[dataIdx]
            #print(f'curveIdx {curveIdx} {pargs.graph is None or str(dataIdx+1) in pargs.graph}')
            if pargs.graph is None or str(dataIdx+1) in pargs.graph:
                curve.setData(x, row*pargs.yscale)
    except KeyboardInterrupt:
        print(' Interrupted')
        serdev.close()
        sys.exit(1)
    except serial.SerialException as e:        
        print(f'ERR: serialException: {e}')
        try: 
            serdev.close()
        except:
            pass
        #printi('Will sleep a second and attempt to re-open the serial link')
        #time.sleep(1)
        #serdev = open_serdev()
        sys.exit(1)

def input_thread():
    global LastCmd
    print('Accepted commands: info, get, set.')
    while(1):
        msg = input()
        if msg.encode() == UpArrow:
            msg = LastCmd
        else:
            LastCmd = msg
        write_serdev(msg)

def write_serdev(msg:str):
    msgb = msg.encode()
    printv(f'cmd:{msgb}')
    #ifdef sendslow
        #if pargs.sendslow:
        #    for chb in msgb:
        #      serdev.write(chb)
        #      time.sleep(0.0001)
        #    serdev.write(b'\r')
        #else:
    #endif
    l = len(msgb)
    msgb += (pargs.lframe - l)*b'\r'
    printv(f'msgb[{len(msgb)}]: {msgb}')
    serdev.write(msgb)
#,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
#``````````````````Main function``````````````````````````````````````````````
if __name__ == "__main__":
    import argparse
    global pargs, plot, TextItem
    parser = argparse.ArgumentParser(description=__doc__
    ,formatter_class=argparse.ArgumentDefaultsHelpFormatter
    ,epilog=f'{sys.argv[0]}: {__version__}')
    parser.add_argument('-B','--black', action='store_true', help=
      'Black background, white foreground for all graphics')
    parser.add_argument('-g', '--graph', default='', nargs='?', help=\
      'Show waveforms on a graph, for example, -g: show all, -g123: only show waveforms 1,2 and 3')
    parser.add_argument('-b', '--baudrate', type=int, default=7372800,# 10000000,
      help='Baud rate of the serial port')
    parser.add_argument('-l', '--lframe',  type=int, default=80, help=\
      'Length of the transmission packet')
    parser.add_argument('-q', '--quiet', action='store_true', help=\
      'Do not print frame statistics')
    parser.add_argument('-n', '--noinit', action='store_true', help=\
      'Do not send any itialization commands to device. (Interactive mode)')
    #ifdef sendslow
    #parser.add_argument('-s', '--sendslow', action='store_true', help=\
    #  'Sending characters one by one')
    #endif
    #parser.add_argument('-t', '--timeout', default=1, help=\
    #  'Timeout for serial receiving')
    parser.add_argument('-u', '--yunits', default='', help=\
      'Units for Y-axis')
    parser.add_argument('-v', '--verbose', nargs='*', help=\
      'Show more log messages (-vv: show even more).')
    parser.add_argument('-x', '--expert', action='store_true', help=\
      'Run in expert mode')
    parser.add_argument('-y', '--yscale', type=float, default=1., help=\
      'Y scale')
    parser.add_argument('-z', '--zoomin', help=\
      'Zoom the application window by a factor')
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0', help=\
      'Port for serial communication')
    pargs = parser.parse_args()
    #breakpoint()

    # check if serdev is not being used by others
    if not pargs.expert:
        import psutil
        for proc in psutil.process_iter():
            if 'mcufecMan' in proc.cmdline():
                printe('Access to MCUFEC is denied, it is used by mcufecMan.')
                sys.exit(1) 
    serdev = open_serdev()
    printi(f"Receive data from {pargs.port}.")

    if pargs.noinit:
        printi('!!!!! Interactive mode, FEC was not forced to start !!!!!')
    else:
        r = get_data('get fecversion')
        if r == 1:
            printe('The FEC expected to be resposive, but it is not')
            sys.exit()
        get_data('get adc_srate')
        get_data('set fec start')

    #```````````````The FEC seems to be alive`````````````````````````````````
    threading.Thread(target=input_thread, daemon=True).start()
    if pargs.graph == '':
        # No plotting.
        while(1):
            read_serial()
            #time.sleep(.1)
    else:
        import pyqtgraph as pg
        if not pargs.black:
            pg.setConfigOption('background', 'w')
            pg.setConfigOption('foreground', 'k')
        from pyqtgraph.Qt import QtCore
        # the --zoom should be handled prior to QtWidgets.QApplication
        try:
            print(f'argv: {sys.argv}')
            idx = sys.argv.index('-z')
            zoom = sys.argv[idx+1]
            print(f'Zoom is set to {zoom}')
            os.environ["QT_SCALE_FACTOR"] = zoom
        except Exception as e:
            #print(f'zoom option is not provided: {e}')
            pass
        app = pg.mkQApp()
        plot = pg.plot()
        plot.setWindowTitle('STM32G431 Scope')
        print(f'ADC sampling rate: {adc_srate} Hz')
        if adc_srate == .9999:
            printw(f'Fail to read ADC sampling rate. It can be recovered using `get adc_srate` command.')
            plot.setLabel('bottom', 'Sample')
        else:
            plot.setLabel('bottom', 'Time', units='S')
        plot.setLabel('left','', units=pargs.yunits)

        legend = pg.LegendItem((80,60), offset=(70,20))
        legend.setParentItem(plot.graphicsItem())

        #text = pg.TextItem(html='<div style="text-align: center"><span style="color: #FFF;">This is the</span><br><span style="color: #FF0; font-size: 16pt;">PEAK</span></div>', anchor=(-0.3,0.5), angle=45, border='w', fill=(0, 0, 255, 100))
        TextItem = pg.TextItem('Aqquisition started', anchor=(0,1))#, ensureInBounds=True)
        TextItem.setParentItem(plot.graphicsItem())
        #plot.addItem(TextItem)
        width = plot.size().width()
        TextItem.setPos(width, 20)
        
        timer = QtCore.QTimer()
        timer.timeout.connect(read_serial)
        timer.start(0)
        app.exec_()
